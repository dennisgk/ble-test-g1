#pragma once
#include <string_view>
#include <vector>
#include <cstdint>
#include <initializer_list>

static const char kData[] = R"(
Vroman Effect::Definition::lower affinity proteins bind...
Vroman Effect::Importance::lower affinity proteins like fibronectin...
Vroman Effect::MW::Smaller..., Higher...

FBR::Protein Adsorption::Definition::the process where proteins stick...
FBR::Acute Inflammation::neutrophils and what they do ...
)";

namespace texttree {

// Simple tree:
// - Takes ONE big external char* blob (non-owning)
// - Parses lines of "A::B::C::value"
// - Uses ONLY std::string_view slices into that blob (no copies of text)
// - Each node is either:
//    * INTERNAL: has children
//    * VALUE: has exactly one value and no children
//
// You can:
// - list children names at a path
// - check if a value exists at a path
// - get value at a path
class TextTree {
public:
  TextTree();
  ~TextTree() = default;

  // Provide the backing text (must outlive this object / queries)
  void set_text(const char* data, std::size_t len);
  void set_text(std::string_view sv);

  // Parse + build tree. Safe to call multiple times after set_text().
  void build();

  // Query
  bool exists(std::initializer_list<std::string_view> path) const;
  bool has_value(std::initializer_list<std::string_view> path) const;

  // Empty if not found or no value at that node.
  std::string_view get_value(std::initializer_list<std::string_view> path) const;

  // Direct child segment names. Empty if not found or node is a value node.
  std::vector<std::string_view> children(std::initializer_list<std::string_view> path) const;

private:
  struct Node {
    std::string_view name;          // segment name
    std::string_view value;         // leaf payload
    bool has_value = false;         // leaf if true
    std::vector<uint32_t> kids;     // indices into nodes
  };

  std::string_view backing_;        // non-owning view of big blob
  std::vector<Node> nodes_;         // nodes_[0] is root

  // helpers
  static bool is_space(char c);
  static std::string_view trim(std::string_view s);

  // Finds child of parent by name, or UINT32_MAX
  uint32_t find_child(uint32_t parent, std::string_view seg) const;

  // Ensure child exists; returns child idx or UINT32_MAX if parent is a value node
  uint32_t get_or_add_child(uint32_t parent, std::string_view seg);

  // Walk path segments, returns node idx or UINT32_MAX
  uint32_t walk(std::initializer_list<std::string_view> path) const;

  // Build from backing_
  void parse_lines();

  // Parses one line (already trimmed + non-empty), returns true if inserted
  bool parse_line(std::string_view line);

  // Splits one segment at a time (no allocations) using "::"
  static bool next_segment(std::string_view& s, std::string_view& seg_out);
};

} // namespace texttree
