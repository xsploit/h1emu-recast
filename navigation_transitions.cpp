#include "navigation_transitions.h"

#include "nav_semantic.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace h1emu::nav {
namespace {

class JsonCursor {
public:
  explicit JsonCursor(std::string text) : text_(std::move(text)) {}

  bool eof() {
    whitespace();
    return offset_ == text_.size();
  }

  bool consume(char expected) {
    whitespace();
    if (offset_ < text_.size() && text_[offset_] == expected) {
      ++offset_;
      return true;
    }
    return false;
  }

  void require(char expected, const char *description) {
    if (!consume(expected))
      fail(std::string("expected ") + description);
  }

  std::string string() {
    whitespace();
    if (offset_ >= text_.size() || text_[offset_] != '"')
      fail("expected string");
    ++offset_;
    std::string value;
    while (offset_ < text_.size()) {
      const unsigned char c = (unsigned char)text_[offset_++];
      if (c == '"')
        return value;
      if (c < 0x20)
        fail("unescaped control character in string");
      if (c != '\\') {
        value.push_back((char)c);
        continue;
      }
      if (offset_ >= text_.size())
        fail("unterminated string escape");
      const char escape = text_[offset_++];
      switch (escape) {
      case '"': value.push_back('"'); break;
      case '\\': value.push_back('\\'); break;
      case '/': value.push_back('/'); break;
      case 'b': value.push_back('\b'); break;
      case 'f': value.push_back('\f'); break;
      case 'n': value.push_back('\n'); break;
      case 'r': value.push_back('\r'); break;
      case 't': value.push_back('\t'); break;
      case 'u':
        // Names and provenance in the canonical file are UTF-8 already. A
        // JSON unicode escape is legal but deliberately rejected here rather
        // than implementing partial surrogate-pair decoding incorrectly.
        fail("unicode escapes are not supported; store UTF-8 directly");
      default: fail("invalid string escape");
      }
    }
    fail("unterminated string");
  }

  double number() {
    whitespace();
    const std::size_t begin = offset_;
    if (peek('-'))
      ++offset_;
    if (peek('0')) {
      ++offset_;
      if (offset_ < text_.size() && isDigit(text_[offset_]))
        fail("leading zero in number");
    } else {
      requireDigits("expected number");
    }
    if (peek('.')) {
      ++offset_;
      requireDigits("expected digits after decimal point");
    }
    if (peek('e') || peek('E')) {
      ++offset_;
      if (peek('+') || peek('-'))
        ++offset_;
      requireDigits("expected exponent digits");
    }
    const std::string token = text_.substr(begin, offset_ - begin);
    errno = 0;
    char *end = nullptr;
    const double value = std::strtod(token.c_str(), &end);
    if (errno == ERANGE || !end || *end != '\0' || !std::isfinite(value))
      fail("number is outside the supported finite range");
    return value;
  }

  bool boolean() {
    whitespace();
    if (text_.compare(offset_, 4, "true") == 0) {
      offset_ += 4;
      return true;
    }
    if (text_.compare(offset_, 5, "false") == 0) {
      offset_ += 5;
      return false;
    }
    fail("expected boolean");
  }

  [[noreturn]] void fail(const std::string &message) const {
    throw std::runtime_error("navigation transitions JSON at byte " +
                             std::to_string(offset_) + ": " + message);
  }

private:
  static bool isDigit(char c) { return c >= '0' && c <= '9'; }

  bool peek(char c) const {
    return offset_ < text_.size() && text_[offset_] == c;
  }

  void requireDigits(const char *message) {
    const std::size_t begin = offset_;
    while (offset_ < text_.size() && isDigit(text_[offset_]))
      ++offset_;
    if (offset_ == begin)
      fail(message);
  }

  void whitespace() {
    while (offset_ < text_.size()) {
      const char c = text_[offset_];
      if (c != ' ' && c != '\t' && c != '\r' && c != '\n')
        break;
      ++offset_;
    }
  }

  std::string text_;
  std::size_t offset_ = 0;
};

std::array<float, 3> parsePoint(JsonCursor &json) {
  std::array<float, 3> point{};
  json.require('[', "'['");
  for (std::size_t i = 0; i < point.size(); ++i) {
    if (i)
      json.require(',', "','");
    const double value = json.number();
    const float narrowed = (float)value;
    if (!std::isfinite(narrowed))
      json.fail("coordinate is outside the finite float range");
    point[i] = narrowed;
  }
  json.require(']', "']'");
  return point;
}

NavigationTransition parseTransition(JsonCursor &json, std::size_t index) {
  NavigationTransition transition;
  transition.sourceIndex = index;
  bool haveName = false;
  bool haveStart = false;
  bool haveEnd = false;
  std::set<std::string> fields;

  json.require('{', "'{'");
  bool first = true;
  while (!json.consume('}')) {
    if (!first)
      json.require(',', "','");
    first = false;
    const std::string key = json.string();
    if (!fields.insert(key).second)
      json.fail("duplicate field '" + key + "'");
    json.require(':', "':'");

    if (key == "name") {
      transition.name = json.string();
      haveName = true;
    } else if (key == "start") {
      transition.start = parsePoint(json);
      haveStart = true;
    } else if (key == "end") {
      transition.end = parsePoint(json);
      haveEnd = true;
    } else if (key == "radius") {
      const double radius = json.number();
      transition.radius = (float)radius;
      if (!std::isfinite(transition.radius) || transition.radius <= 0.0f)
        json.fail("radius must be a positive finite float");
    } else if (key == "bidirectional") {
      transition.bidirectional = json.boolean();
    } else if (key == "kind" || key == "source") {
      (void)json.string(); // authored provenance, not Detour behavior
    } else {
      json.fail("unknown transition field '" + key + "'");
    }
  }

  if (!haveName || transition.name.empty())
    json.fail("transition requires a non-empty name");
  if (!haveStart || !haveEnd)
    json.fail("transition requires start and end points");
  if (!transition.bidirectional)
    json.fail("navigation transition schema v1 requires bidirectional=true");
  if (transition.start == transition.end)
    json.fail("transition endpoints must differ");
  return transition;
}

} // namespace

std::vector<NavigationTransition>
loadNavigationTransitions(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::runtime_error("cannot open navigation transitions: " + path);
  std::ostringstream bytes;
  bytes << input.rdbuf();
  if (input.bad())
    throw std::runtime_error("cannot read navigation transitions: " + path);

  JsonCursor json(bytes.str());
  std::vector<NavigationTransition> transitions;
  std::set<std::string> names;
  json.require('[', "'['");
  bool first = true;
  while (!json.consume(']')) {
    if (!first)
      json.require(',', "','");
    first = false;
    NavigationTransition transition =
        parseTransition(json, transitions.size());
    if (!names.insert(transition.name).second)
      json.fail("duplicate transition name '" + transition.name + "'");
    transitions.push_back(std::move(transition));
  }
  if (!json.eof())
    json.fail("unexpected trailing content");
  if (transitions.size() >
      (std::size_t)(0xffffffffu - NAVIGATION_TRANSITION_USER_ID_BASE))
    throw std::runtime_error("too many navigation transitions for user-id range");
  return transitions;
}

NavigationTransitionBinding buildNavigationTransitionBinding(
    const std::vector<NavigationTransition> &transitions,
    const float *boundsMin, const float *boundsMax, float yPadding) {
  if ((boundsMin == nullptr) != (boundsMax == nullptr))
    throw std::invalid_argument("transition ownership requires both bounds");
  if (!std::isfinite(yPadding) || yPadding < 0.0f)
    throw std::invalid_argument("transition ownership padding is invalid");

  NavigationTransitionBinding result;
  result.vertices.reserve(transitions.size() * 6);
  result.radii.reserve(transitions.size());
  result.directions.reserve(transitions.size());
  result.areas.reserve(transitions.size());
  result.flags.reserve(transitions.size());
  result.userIds.reserve(transitions.size());

  for (const NavigationTransition &transition : transitions) {
    if (boundsMin &&
        !(transition.start[0] >= boundsMin[0] &&
          transition.start[0] < boundsMax[0] &&
          transition.start[1] >= boundsMin[1] - yPadding &&
          transition.start[1] <= boundsMax[1] + yPadding &&
          transition.start[2] >= boundsMin[2] &&
          transition.start[2] < boundsMax[2]))
      continue;
    result.vertices.insert(result.vertices.end(), transition.start.begin(),
                           transition.start.end());
    result.vertices.insert(result.vertices.end(), transition.end.begin(),
                           transition.end.end());
    result.radii.push_back(transition.radius);
    result.directions.push_back(transition.bidirectional ? 1u : 0u);
    result.areas.push_back(NAV_AREA_THRESHOLD);
    result.flags.push_back(flagsForArea(NAV_AREA_THRESHOLD));
    result.userIds.push_back(NAVIGATION_TRANSITION_USER_ID_BASE +
                             (unsigned int)transition.sourceIndex);
  }
  return result;
}

} // namespace h1emu::nav
