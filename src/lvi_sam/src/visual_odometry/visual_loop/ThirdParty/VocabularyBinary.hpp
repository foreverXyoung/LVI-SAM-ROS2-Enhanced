#ifndef VocabularyBinary_hpp
#define VocabularyBinary_hpp

#include <cstdint>
#include <cstddef>
#include <fstream>
#include <string>
#include <type_traits>

namespace VINSLoop {
    
struct Node {
    int32_t nodeId;
    int32_t parentId;
    double weight;
    uint64_t descriptor[4];
};

struct Word {
    int32_t nodeId;
    int32_t wordId;
};

struct Vocabulary {
    int32_t k;
    int32_t L;
    int32_t scoringType;
    int32_t weightingType;
    
    int32_t nNodes;
    int32_t nWords;
    
    Node* nodes;
    Word* words;
    
    Vocabulary();
    ~Vocabulary();
    
    void serialize(std::ofstream& stream);
    void deserialize(std::ifstream& stream);
    
    inline static constexpr std::size_t staticDataSize() {
        return sizeof(Vocabulary) - sizeof(Node*) - sizeof(Word*);
    }
};

// The legacy binary vocabulary format writes only the scalar prefix of this
// structure. Keep the two owning pointers outside that serialized prefix on
// every supported ABI; otherwise deserialization could overwrite live heap
// addresses with pointer bytes stored in the vocabulary file.
static_assert(std::is_standard_layout<Vocabulary>::value,
              "Vocabulary must remain standard-layout");
static_assert(offsetof(Vocabulary, nodes) == Vocabulary::staticDataSize(),
              "Vocabulary binary header must end before owning pointers");

}

#endif /* VocabularyBinary_hpp */
