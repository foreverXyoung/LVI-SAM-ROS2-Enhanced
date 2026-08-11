#include "VocabularyBinary.hpp"
#include <opencv2/core/core.hpp>
#include <limits>
#include <memory>
#include <stdexcept>
using namespace std;

VINSLoop::Vocabulary::Vocabulary()
: nNodes(0), nWords(0), nodes(nullptr), words(nullptr) {
}

VINSLoop::Vocabulary::~Vocabulary() {
    if (nodes != nullptr) {
        delete [] nodes;
        nodes = nullptr;
    }
    
    if (words != nullptr) {
        delete [] words;
        words = nullptr;
    }
}
    
void VINSLoop::Vocabulary::serialize(ofstream& stream) {
    stream.write((const char *)this, staticDataSize());
    stream.write((const char *)nodes, sizeof(Node) * nNodes);
    stream.write((const char *)words, sizeof(Word) * nWords);
}
    
void VINSLoop::Vocabulary::deserialize(ifstream& stream) {
    stream.read((char *)this, staticDataSize());
    if (!stream || k <= 0 || L <= 0 || nNodes <= 0 || nWords <= 0 ||
        nWords > nNodes || nNodes > 5000000 || nWords > 5000000) {
        throw std::runtime_error("Invalid or unsupported BRIEF vocabulary header");
    }

    const auto payloadStart = stream.tellg();
    stream.seekg(0, std::ios::end);
    const auto payloadEnd = stream.tellg();
    stream.seekg(payloadStart);
    if (payloadStart < 0 || payloadEnd < payloadStart) {
        throw std::runtime_error("Unable to determine BRIEF vocabulary size");
    }
    const std::uint64_t requiredBytes =
        static_cast<std::uint64_t>(sizeof(Node)) *
            static_cast<std::uint64_t>(nNodes) +
        static_cast<std::uint64_t>(sizeof(Word)) *
            static_cast<std::uint64_t>(nWords);
    const std::uint64_t availableBytes = static_cast<std::uint64_t>(
        payloadEnd - payloadStart);
    if (requiredBytes > availableBytes) {
        throw std::runtime_error("Truncated BRIEF vocabulary payload");
    }

    auto loadedNodes = std::make_unique<Node[]>(nNodes);
    auto loadedWords = std::make_unique<Word[]>(nWords);
    stream.read(
        reinterpret_cast<char*>(loadedNodes.get()),
        static_cast<std::streamsize>(sizeof(Node)) * nNodes);
    stream.read(
        reinterpret_cast<char*>(loadedWords.get()),
        static_cast<std::streamsize>(sizeof(Word)) * nWords);
    if (!stream) {
        throw std::runtime_error("Failed to read BRIEF vocabulary payload");
    }

    delete[] nodes;
    delete[] words;
    nodes = loadedNodes.release();
    words = loadedWords.release();
}
