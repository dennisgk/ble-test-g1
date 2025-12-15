#include "TextTree.hpp"

namespace texttree {

TextTree::TextTree() {
  nodes_.push_back(Node{}); // root
}

void TextTree::set_text(const char* data, std::size_t len) {
  backing_ = std::string_view(data, len);
}

void TextTree::set_text(std::string_view sv) {
  backing_ = sv;
}

void TextTree::build() {
  nodes_.clear();
  nodes_.push_back(Node{}); // root
  parse_lines();
}

bool TextTree::exists(std::initializer_list<std::string_view> path) const {
  return walk(path) != UINT32_MAX;
}

bool TextTree::has_value(std::initializer_list<std::string_view> path) const {
  uint32_t idx = walk(path);
  return idx != UINT32_MAX && nodes_[idx].has_value;
}

std::string_view TextTree::get_value(std::initializer_list<std::string_view> path) const {
  uint32_t idx = walk(path);
  if (idx == UINT32_MAX) return {};
  return nodes_[idx].has_value ? nodes_[idx].value : std::string_view{};
}

std::vector<std::string_view> TextTree::children(std::initializer_list<std::string_view> path) const {
  std::vector<std::string_view> out;
  uint32_t idx = walk(path);
  if (idx == UINT32_MAX) return out;

  const Node& n = nodes_[idx];
  if (n.has_value) return out; // leaf => no children

  out.reserve(n.kids.size());
  for (uint32_t c : n.kids) out.push_back(nodes_[c].name);
  return out;
}

// ---------------- helpers ----------------

bool TextTree::is_space(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

std::string_view TextTree::trim(std::string_view s) {
  while (!s.empty() && is_space(s.front())) s.remove_prefix(1);
  while (!s.empty() && is_space(s.back()))  s.remove_suffix(1);
  return s;
}

uint32_t TextTree::find_child(uint32_t parent, std::string_view seg) const {
  const Node& p = nodes_[parent];
  for (uint32_t idx : p.kids) {
    if (nodes_[idx].name == seg) return idx;
  }
  return UINT32_MAX;
}

uint32_t TextTree::get_or_add_child(uint32_t parent, std::string_view seg) {
  // parent can't have children if it's a value node
  if (nodes_[parent].has_value) return UINT32_MAX;

  uint32_t existing = find_child(parent, seg);
  if (existing != UINT32_MAX) return existing;

  // Create child node (may reallocate nodes_)
  uint32_t child_idx = static_cast<uint32_t>(nodes_.size());
  nodes_.push_back(Node{});
  nodes_[child_idx].name = seg;

  // IMPORTANT: re-acquire parent AFTER push_back (avoid dangling ref)
  nodes_[parent].kids.push_back(child_idx);
  return child_idx;
}

uint32_t TextTree::walk(std::initializer_list<std::string_view> path) const {
  uint32_t cur = 0;
  for (auto seg : path) {
    seg = trim(seg);
    if (seg.empty()) return UINT32_MAX;
    cur = find_child(cur, seg);
    if (cur == UINT32_MAX) return UINT32_MAX;
  }
  return cur;
}

// Splits one segment from s by "::" without allocating.
// After call:
// - seg_out is the next segment
// - s is advanced past "::" (or becomes empty)
bool TextTree::next_segment(std::string_view& s, std::string_view& seg_out) {
  if (s.empty()) return false;
  size_t pos = s.find("::");
  if (pos == std::string_view::npos) {
    seg_out = s;
    s = std::string_view{};
    return true;
  }
  seg_out = s.substr(0, pos);
  s.remove_prefix(pos + 2);
  return true;
}

void TextTree::parse_lines() {
  std::string_view text = backing_;

  size_t i = 0;
  while (i < text.size()) {
    size_t end = text.find('\n', i);
    if (end == std::string_view::npos) end = text.size();

    std::string_view line = text.substr(i, end - i);
    if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
    line = trim(line);

    if (!line.empty()) {
      (void)parse_line(line); // ignore bad lines
    }

    i = (end == text.size()) ? text.size() : end + 1;
  }
}

bool TextTree::parse_line(std::string_view line) {
  // We interpret the last segment as VALUE, everything before as PATH.
  // Format requires at least one "::" => at least 2 segments.
  //
  // We'll walk segments one by one without allocating a vector:
  // - keep a rolling "prev segment"
  // - we can't know which is last until we try to read next
  //
  // Approach:
  //   read first seg into curSeg
  //   while (read next seg into nextSeg):
  //     if (after reading nextSeg, there is MORE to read later) => curSeg is a PATH segment
  //     else => nextSeg is last? Actually with rolling we handle at end:
  //
  // Simpler:
  //   Extract all segments into a small local vector? (alloc)
  // But you want simple + low overhead; we'll do a no-alloc two-pass:
  //   1) find last "::" position, split path/value
  //   2) parse path side by scanning segments

  size_t last = line.rfind("::");
  if (last == std::string_view::npos) return false; // no delimiter

  std::string_view pathPart = trim(line.substr(0, last));
  std::string_view valuePart = trim(line.substr(last + 2));
  if (pathPart.empty() || valuePart.empty()) return false;

  uint32_t cur = 0;

  // Parse path segments from pathPart
  std::string_view rest = pathPart;
  while (!rest.empty()) {
    std::string_view seg;
    if (!next_segment(rest, seg)) break;
    seg = trim(seg);
    if (seg.empty()) return false;

    cur = get_or_add_child(cur, seg);
    if (cur == UINT32_MAX) return false; // tried to add under leaf
  }

  // Set value at node cur (must be leaf)
  Node& n = nodes_[cur];
  if (!n.kids.empty()) {
    // internal already => violates invariant; ignore
    return false;
  }
  n.has_value = true;
  n.value = valuePart;
  return true;
}

} // namespace texttree
