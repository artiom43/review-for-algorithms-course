#include <algorithm>
#include <cstring>
#include <deque>
#include <iostream>
#include <iterator>

#include <map>
#include <list>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>


template<class Iterator>
class IteratorRange {
public:
    IteratorRange(Iterator begin, Iterator end) : begin_(begin), end_(end) {}

    Iterator begin() const { return begin_; }

    Iterator end() const { return end_; }

private:
    Iterator begin_, end_;
};

namespace traverses {

// Traverses the connected component in a breadth-first order
// from the vertex 'origin_vertex'.
// Refer to
// https://goo.gl/0qYXzC
// for the visitor events.
    template<class Vertex, class Graph, class Visitor>
    void BreadthFirstSearch(Vertex origin_vertex, const Graph &graph,
                            Visitor visitor) {
        std::deque<Vertex> deque_of_vertexs = {origin_vertex};
        std::unordered_set<Vertex> used_vertexs;
        used_vertexs.insert(origin_vertex);
        visitor.DiscoverVertex(origin_vertex);
        while (!deque_of_vertexs.empty()) {
            visitor.ExamineVertex(deque_of_vertexs.front());
            for (auto outgoing_edge: OutgoingEdges(graph, deque_of_vertexs.front())) {
                visitor.ExamineEdge(outgoing_edge);
                auto result_of_get_target = GetTarget(graph, outgoing_edge);
                if (used_vertexs.count(result_of_get_target) == 0) {
                    visitor.DiscoverVertex(result_of_get_target);
                    deque_of_vertexs.emplace_back(result_of_get_target);
                    used_vertexs.insert(result_of_get_target);
                }
            }
            deque_of_vertexs.pop_front();
        }
    }


// See "Visitor Event Points" on
// https://goo.gl/wtAl0y
    template<class Vertex, class Edge>
    class BfsVisitor {
    public:
        virtual void DiscoverVertex(Vertex /*vertex*/) {}

        virtual void ExamineEdge(const Edge & /*edge*/) {}

        virtual void ExamineVertex(Vertex /*vertex*/) {}

        virtual ~BfsVisitor() = default;
    };

} // namespace traverses

namespace aho_corasick {

    struct AutomatonNode {
        AutomatonNode() : suffix_link(nullptr), terminal_link(nullptr) {}

        // Stores ids of strings which are ended at this node.
        std::vector<size_t> terminated_string_ids;
        // Stores tree structure of nodes.
        std::map<char, AutomatonNode> trie_transitions;

        // Stores cached transitions of the automaton, contains
        // only pointers to the elements of trie_transitions.
        std::map<char, AutomatonNode *> automaton_transitions_cache;
        AutomatonNode *suffix_link;
        AutomatonNode *terminal_link;
    };

// Returns a corresponding trie transition 'nullptr' otherwise.
    AutomatonNode *GetTrieTransition(AutomatonNode *node, char character) {
        auto result_of_find_character = (node->trie_transitions).find(character);
        if (result_of_find_character != node->trie_transitions.end()) {
            return &(result_of_find_character->second);
        }
        return nullptr;
    }

// Returns an automaton transition, updates 'node->automaton_transitions_cache'
// if necessary.
// Provides constant amortized runtime.
    AutomatonNode *GetAutomatonTransition(AutomatonNode *node,
                                          const AutomatonNode *root,
                                          char character) {
        if (node->automaton_transitions_cache.find(character) !=
            node->automaton_transitions_cache.end()) {
            return node->automaton_transitions_cache[character];
        }
        auto result_get_trie_transition = GetTrieTransition(node, character);
        if (result_get_trie_transition != nullptr) {
            node->automaton_transitions_cache[character] = result_get_trie_transition;
            return node->automaton_transitions_cache[character];
        }
        if (root != node) {
            node->automaton_transitions_cache[character] =
                    GetAutomatonTransition(node->suffix_link, root, character);
            return node->automaton_transitions_cache[character];
        }
        return node;
    }

    namespace internal {

        class AutomatonGraph {
        public:
            struct Edge {
                Edge(AutomatonNode *source, AutomatonNode *target, char character)
                        : source(source), target(target), character(character) {}

                AutomatonNode *source;
                AutomatonNode *target;
                char character;
            };
        };

        std::vector<typename AutomatonGraph::Edge> OutgoingEdges(
                const AutomatonGraph & /*graph*/, AutomatonNode *vertex) {
            std::vector<typename AutomatonGraph::Edge> list_of_outgoing_edges;
            for (auto &pair: vertex->trie_transitions) {
                list_of_outgoing_edges.emplace_back(vertex, &pair.second,
                                                    pair.first);
            }
            return list_of_outgoing_edges;
        }

        AutomatonNode *GetTarget(const AutomatonGraph & /*graph*/,
                                 const AutomatonGraph::Edge &edge) {
            return edge.target;
        }

        class SuffixLinkCalculator
                : public traverses::BfsVisitor<AutomatonNode *, AutomatonGraph::Edge> {
        public:
            explicit SuffixLinkCalculator(AutomatonNode *root) : root_(root) {}

            void ExamineVertex(AutomatonNode *node) override {
                if (node == root_) {
                    node->suffix_link = root_;
                    return;
                }
            }

            void ExamineEdge(const AutomatonGraph::Edge &edge) override {
                if (edge.source == root_) {
                    edge.target->suffix_link = root_;
                    return;
                }
                edge.target->suffix_link = GetAutomatonTransition(edge.source->suffix_link,
                                                                  root_, edge.character);
            }

        private:
            AutomatonNode *root_;
        };

        class TerminalLinkCalculator
                : public traverses::BfsVisitor<AutomatonNode *, AutomatonGraph::Edge> {
        public:
            explicit TerminalLinkCalculator(AutomatonNode *root) : root_(root) {}

            void DiscoverVertex(AutomatonNode *node) override {
                if (node == root_) {
                    node->terminal_link = root_;
                    return;
                }
                if (node->suffix_link->terminated_string_ids.empty()) {
                    node->terminal_link = node->suffix_link->terminal_link;
                } else {
                    node->terminal_link = node->suffix_link;
                }
            }

        private:
            AutomatonNode *root_;
        };
}   // namespace internal


    class NodeReference {
    public:
        NodeReference() : node_(nullptr), root_(nullptr) {}

        NodeReference(AutomatonNode *node, AutomatonNode *root)
                : node_(node), root_(root) {}

        NodeReference Next(char character) const {
            return {GetAutomatonTransition(
                    node_, root_, character), root_};
        }

        template<class Callback>
        void GenerateMatches(Callback on_match) const {
            for (auto matched_id: node_->terminated_string_ids) {
                on_match(matched_id);
            }
            if (node_ != root_) {
                TerminalLink().GenerateMatches(on_match);
            }
        }

        bool IsTerminal() const {
            return node_->terminated_string_ids.empty();
        }

        explicit operator bool() const { return node_ != nullptr; }

        bool operator==(NodeReference other) const {

            return node_ == other.node_;
        }

    private:
        using TerminatedStringIterator = std::vector<int>::const_iterator;
        using TerminatedStringIteratorRange = IteratorRange<TerminatedStringIterator>;

        NodeReference TerminalLink() const {
            return {node_->terminal_link, root_};
        }

        TerminatedStringIteratorRange TerminatedStringIds() const;

        AutomatonNode *node_;
        AutomatonNode *root_;
    };

    class AutomatonBuilder;

    class Automaton {
    public:
        Automaton() = default;

        Automaton(const Automaton &) = delete;

        Automaton &operator=(const Automaton &) = delete;

        NodeReference Root() {
            return NodeReference(&root_, &root_);
        }

    private:
        AutomatonNode root_;

        friend class AutomatonBuilder;
    };

    class AutomatonBuilder {
    public:
        void Add(const std::string &string, size_t id) {
            words_.push_back(string);
            ids_.push_back(id);
        }

        std::unique_ptr<Automaton> Build() {
            auto automaton = std::make_unique<Automaton>();
            BuildTrie(words_, ids_, automaton.get());
            BuildSuffixLinks(automaton.get());
            BuildTerminalLinks(automaton.get());
            return automaton;
        }

    private:
        static void BuildTrie(const std::vector<std::string> &words,
                              const std::vector<size_t> &ids, Automaton *automaton) {
            for (size_t i = 0; i < words.size(); ++i) {
                AddString(&automaton->root_, ids[i], words[i]);
            }
        }

        static void AddString(AutomatonNode *root, size_t string_id,
                              const std::string &string) {
            AutomatonNode *current_automation_node = root;
            for (auto character: string) {
                current_automation_node = &current_automation_node->trie_transitions[character];
            }
            current_automation_node->terminated_string_ids.push_back(string_id);
        }

        static void BuildSuffixLinks(Automaton *automaton) {
            traverses::BreadthFirstSearch(&automaton->root_, internal::AutomatonGraph(),
                                          internal::SuffixLinkCalculator(&automaton->root_));
        }

        static void BuildTerminalLinks(Automaton *automaton) {
            traverses::BreadthFirstSearch(&automaton->root_, internal::AutomatonGraph(),
                                          internal::TerminalLinkCalculator(&automaton->root_));
        }

        std::vector<std::string> words_;
        std::vector<size_t> ids_;
    };

} // namespace aho_corasick

// Consecutive delimiters are not grouped together and are deemed
// to delimit empty strings
template<class Predicate>
std::vector<std::string> Split(const std::string &string,
                               Predicate is_delimiter) {
    std::vector<std::string> vector_of_resulted_strings;
    auto current_left_iterator = string.begin();
    do {
        std::string::const_iterator current_right_iterator =
                std::find_if(current_left_iterator, string.end(), is_delimiter);
        vector_of_resulted_strings.emplace_back(current_left_iterator,
                                                current_right_iterator);
        current_left_iterator = current_right_iterator;
        ++current_left_iterator;
    } while (current_left_iterator - 1 < string.end());
    return vector_of_resulted_strings;
}

// Wildcard is a character that may be substituted
// for any of all possible characters.

class WildcardMatcher {
public:
    WildcardMatcher() : number_of_words_(0), pattern_length_(0) {}

    WildcardMatcher static BuildFor(const std::string &pattern, char wildcard) {
        std::vector<std::string> vector_of_splited_string =
                Split(pattern, [&wildcard](char x) { return x == wildcard; });
        auto wildcard_matcher = WildcardMatcher();
        aho_corasick::AutomatonBuilder side_aho_corasick = aho_corasick::AutomatonBuilder();
        size_t current_position = 0;
        size_t number_of_non_empty_strings = 0;
        for (auto &current_string: vector_of_splited_string) {
            if (!current_string.empty()) {
                side_aho_corasick.Add(current_string,
                                      current_position + current_string.size() - 1);
                ++number_of_non_empty_strings;
            }
            current_position += current_string.size() + 1;
        }
        wildcard_matcher.aho_corasick_automaton_ = side_aho_corasick.Build();
        wildcard_matcher.pattern_length_ = pattern.size();
        wildcard_matcher.Reset();
        wildcard_matcher.number_of_words_ = number_of_non_empty_strings;
        return wildcard_matcher;
    }

    // Resets the matcher. Call allows to abandon all data which was already
    // scanned,
    // a new stream can be scanned afterwards.
    void Reset() {
        state_ = aho_corasick_automaton_->Root();
        words_occurrences_by_position_.clear();
        words_occurrences_by_position_.resize(pattern_length_);
    }


    template<class Callback>
    void Scan(char character, Callback on_match) {
        state_ = state_.Next(character);
        UpdateWordOccurrencesCounters();
        if (words_occurrences_by_position_[0] == number_of_words_) {
            on_match();
        }
        ShiftWordOccurrencesCounters();
    }

private:
    // Storing only O(|pattern|) elements allows us
    // to consume only O(|pattern|) memory for matcher.
    std::deque<size_t> words_occurrences_by_position_;
    aho_corasick::NodeReference state_;
    size_t number_of_words_;
    size_t pattern_length_;
    std::unique_ptr<aho_corasick::Automaton> aho_corasick_automaton_;

    void UpdateWordOccurrencesCounters() {
        state_.GenerateMatches([this](size_t string_id) {
            ++words_occurrences_by_position_[pattern_length_ - 1 - string_id];
        });
    }

    void ShiftWordOccurrencesCounters() {
        words_occurrences_by_position_.pop_front();
        words_occurrences_by_position_.push_back(0);
    }
};

std::string ReadString(std::istream &input_stream) {
    std::string input_string;
    std::getline(input_stream, input_string);
    return input_string;
}


// Returns positions of the first character of an every match.
std::vector<int> FindFuzzyMatches(const std::string &pattern_with_wildcards,
                                  const std::string &text, char wildcard) {
    WildcardMatcher wildcard_matcher = WildcardMatcher().BuildFor(
            pattern_with_wildcards, wildcard);
    std::vector<size_t> occurrences;
    for (size_t index = 0; index < text.size(); ++index) {
        wildcard_matcher.Scan(text[index],
                              [&occurrences, &pattern_with_wildcards, index]() {
                                  occurrences.push_back(index - pattern_with_wildcards.size() + 1);
                              });
    }
    auto left_iterator = std::find_if(occurrences.begin(), occurrences.end(),
                                      [](int number) {
                                          return number >= 0;
                                      });
    return {left_iterator, occurrences.end()};
}

void Print(const std::vector<int> &sequence) {
    std::cout << sequence.size() << "\n";
    for (auto position: sequence) {
        std::cout << position << " ";
    }
}
