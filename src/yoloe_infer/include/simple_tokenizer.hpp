#ifndef SIMPLE_TOKENIZER_HPP
#define SIMPLE_TOKENIZER_HPP

#include <string>
#include <vector>
#include <map>
#include <set>
#include <regex>
#include <memory>
#include <nlohmann/json.hpp>

namespace yoloe_infer {

class SimpleTokenizer {
public:
    SimpleTokenizer(const std::string& data_path = "");
    ~SimpleTokenizer() = default;

    std::vector<int> encode(const std::string& text);
    std::string decode(const std::vector<int>& tokens);
    std::vector<int> operator()(const std::string& text, int context_length = 77);
    std::vector<std::vector<int>> operator()(const std::vector<std::string>& texts, int context_length = 77);

private:
    std::map<int, std::string> bytes_to_unicode();
    std::set<std::pair<std::string, std::string>> get_pairs(const std::vector<std::string>& word);
    std::string bpe(const std::string& token);
    std::string whitespace_clean(std::string text);
    
    std::map<std::string, int> encoder_;
    std::map<int, std::string> decoder_;
    std::map<std::pair<std::string, std::string>, int> bpe_ranks_;
    std::map<std::string, std::string> cache_;
    std::map<int, std::string> byte_encoder_; // map byte value (0-255) to unicode string
    std::map<std::string, int> byte_decoder_; // invalid utf-8 byte string to original byte value

    std::regex pat_;
    int context_length_ = 77;
    int sot_token_ = 49406;
    int eot_token_ = 49407;
};

} // namespace yoloe_infer

#endif // SIMPLE_TOKENIZER_HPP
