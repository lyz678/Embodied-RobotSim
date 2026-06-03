#include "simple_tokenizer.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <codecvt>
#include <locale>
#include <algorithm>
#include <filesystem>
#include <zlib.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace yoloe_infer {

// Helper to decompress gzip data
std::string decompress_gzip(const std::string& str) {
    z_stream zs;
    memset(&zs, 0, sizeof(zs));

    if (inflateInit2(&zs, 16 + MAX_WBITS) != Z_OK)
        throw std::runtime_error("inflateInit2 failed while decompressing.");

    zs.next_in = (Bytef*)str.data();
    zs.avail_in = str.size();

    int ret;
    char outbuffer[32768];
    std::string outstring;

    do {
        zs.next_out = reinterpret_cast<Bytef*>(outbuffer);
        zs.avail_out = sizeof(outbuffer);

        ret = inflate(&zs, Z_NO_FLUSH);

        if (outstring.size() < zs.total_out) {
            outstring.append(outbuffer, zs.total_out - outstring.size());
        }

    } while (ret == Z_OK);

    inflateEnd(&zs);

    if (ret != Z_STREAM_END) {
        throw std::runtime_error("Exception during zlib decompression: " + std::to_string(ret));
    }

    return outstring;
}

SimpleTokenizer::SimpleTokenizer(const std::string& data_path) {
    std::string path = data_path;
    if (path.empty()) {
        // Default path relative to the installed package
        // Try to find it in the installed share directory or local development path
        // Try using ament_index_cpp to find the package share directory
        try {
            path = ament_index_cpp::get_package_share_directory("yoloe_infer") + "/models/tokenizer_data.json.gz";
        } catch (...) {
            // Fallback to local development path
            path = "src/yoloe_infer/models/tokenizer_data.json.gz";
        }
    }

    // Load and decompress
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open tokenizer file: " + path + " (errno: " + std::to_string(errno) + ")");
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    
    // Check if gzipped (magic number 1f 8b)
    if (content.size() > 2 && (uint8_t)content[0] == 0x1f && (uint8_t)content[1] == 0x8b) {
        content = decompress_gzip(content);
    }

    using json = nlohmann::json;
    json data = json::parse(content);

    // Context length
    context_length_ = data.value("context_length", 77);
    sot_token_ = data.value("sot_token", 49406);
    eot_token_ = data.value("eot_token", 49407);

    // Encoder/Decoder
    for (auto& element : data["vocab"].items()) {
        encoder_[element.key()] = element.value();
        decoder_[element.value()] = element.key();
    }

    // Merges
    auto merges = data["merges"];
    int i = 0;
    for (const auto& merge : merges) {
        std::string m = merge;
        size_t space_pos = m.find(' ');
        if (space_pos != std::string::npos) {
            std::string first = m.substr(0, space_pos);
            std::string second = m.substr(space_pos + 1);
            bpe_ranks_[{first, second}] = i++;
        }
    }

    // Bytes to Unicode
    auto b2u = bytes_to_unicode();
    for (auto const& [key, val] : b2u) {
        byte_decoder_[val] = key;
    }
    
    // Regex
    // <\|startoftext\|>|<\|endoftext\|>|'s|'t|'re|'ve|'m|'ll|'d|[\p{L}]+|[\p{N}]|[^\s\p{L}\p{N}]+
    // C++ std::regex doesn't support \p{L} well without ICU or Boost. 
    // We will use a simplified approximate regex or std::wregex if needed.
    // However, the python regex library is powerful. 
    // For standard C++, we might need to approximate or use a simple split if complex regex is not critical.
    // The python regex was: r"""<\|startoftext\|>|<\|endoftext\|>|'s|'t|'re|'ve|'m|'ll|'d|[\p{L}]+|[\p{N}]|[^\s\p{L}\p{N}]+"""
    // Simplified for standard regex (no unicode properties \p):
    // We'll trust the input is mostly english simple chars for now, or use a basic tokenization.
    // 's|'t|'re|'ve|'m|'ll|'d| [a-zA-Z]+ | [0-9] | [^ \n\r\t\a-zA-Z0-9]+
    pat_ = std::regex(R"(<\|startoftext\|>|<\|endoftext\|>|'s|'t|'re|'ve|'m|'ll|'d|[a-zA-Z]+|[0-9]|[^ \n\r\t\f\v a-zA-Z0-9]+)", std::regex_constants::icase);
}

std::map<int, std::string> SimpleTokenizer::bytes_to_unicode() {
    std::map<int, std::string> bs_cs;
    std::vector<int> bs;
    
    // ! to ~
    for (int i = '!'; i <= '~'; ++i) bs.push_back(i);
    // ¡ to ¬
    for (int i = 161; i <= 172; ++i) bs.push_back(i);
    // ® to ÿ
    for (int i = 174; i <= 255; ++i) bs.push_back(i);

    std::vector<int> cs = bs;
    int n = 0;
    for (int b = 0; b < 256; ++b) {
        bool found = false;
        for (int v : bs) if (v == b) found = true;
        if (!found) {
            bs.push_back(b);
            cs.push_back(256 + n);
            n++;
        }
    }
    
    for (size_t i = 0; i < bs.size(); ++i) {
        // Convert unicode codepoint cs[i] to utf-8 string
        int cp = cs[i];
        std::string utf8_char;
        if (cp < 0x80) {
            utf8_char += (char)cp;
        } else if (cp < 0x800) {
            utf8_char += (char)(0xC0 | (cp >> 6));
            utf8_char += (char)(0x80 | (cp & 0x3F));
        } // We only go up to 256 + 256 ~ 512, so 2 bytes is enough (max 0x7FF)
        
        bs_cs[bs[i]] = utf8_char;
        
        // Populate member map
        byte_encoder_[bs[i]] = utf8_char;
    }
    return bs_cs;
}

std::set<std::pair<std::string, std::string>> SimpleTokenizer::get_pairs(const std::vector<std::string>& word) {
    std::set<std::pair<std::string, std::string>> pairs;
    if (word.empty()) return pairs;
    for (size_t i = 0; i < word.size() - 1; ++i) {
        pairs.insert({word[i], word[i+1]});
    }
    return pairs;
}

std::string SimpleTokenizer::bpe(const std::string& token) {
    if (cache_.count(token)) return cache_[token];

    // Split token into characters (assuming utf-8 chars logic, here simplified to bytes mapping)
    std::vector<std::string> word;
    // The input token is already mapped bytes-to-unicode chars.
    // We treat each "char" as a unit. Since our mapping ensures 1-to-1 mostly for the base range.
    // Actually, in C++, iterating a utf-8 string by character is tricky.
    // But since `token` comes from `encode` where we mapped bytes to mapped-unicode-strings,
    // lets be careful.
    
    // In Python: tuple(token[:-1]) + (token[-1] + '</w>',)
    // We need to iterate over the utf-8 characters of 'token'.
    // Since our mapped characters are up to 2 bytes, we can scan.
    
    int i = 0;
    while (i < (int)token.length()) {
        int len = 1;
        if ((token[i] & 0x80) == 0) len = 1;
        else if ((token[i] & 0xE0) == 0xC0) len = 2;
        else if ((token[i] & 0xF0) == 0xE0) len = 3;
        else if ((token[i] & 0xF8) == 0xF0) len = 4;
        
        word.push_back(token.substr(i, len));
        i += len;
    }

    if (!word.empty()) {
        word.back() += "</w>";
    }
    
    std::set<std::pair<std::string, std::string>> pairs = get_pairs(word);
    
    if (pairs.empty()) {
        return token + "</w>";
    }

    while (true) {
        // Find min pair by rank
        auto best_pair_it = pairs.end();
        int min_rank = std::numeric_limits<int>::max();
        std::pair<std::string, std::string> best_pair;

        for (const auto& pair : pairs) {
            if (bpe_ranks_.count(pair)) {
                int rank = bpe_ranks_.at(pair);
                if (rank < min_rank) {
                    min_rank = rank;
                    best_pair = pair;
                    best_pair_it = pairs.find(pair); // just to mark found
                }
            }
        }
        
        if (min_rank == std::numeric_limits<int>::max()) break;

        std::string first = best_pair.first;
        std::string second = best_pair.second;
        std::vector<std::string> new_word;
        
        size_t j = 0;
        while (j < word.size()) {
            // Find occurrence of first
            auto it = std::find(word.begin() + j, word.end(), first);
            if (it == word.end()) {
                new_word.insert(new_word.end(), word.begin() + j, word.end());
                break;
            }
            
            size_t idx = std::distance(word.begin(), it);
            new_word.insert(new_word.end(), word.begin() + j, it);
            j = idx;
            
            if (j < word.size() - 1 && word[j] == first && word[j+1] == second) {
                new_word.push_back(first + second);
                j += 2;
            } else {
                new_word.push_back(word[j]);
                j += 1;
            }
        }
        
        word = new_word;
        if (word.size() == 1) break;
        pairs = get_pairs(word);
    }
    
    // Join with spaces
    std::string result;
    for (size_t k = 0; k < word.size(); ++k) {
        if (k > 0) result += " ";
        result += word[k];
    }
    cache_[token] = result;
    return result;
}

std::string SimpleTokenizer::whitespace_clean(std::string text) {
    // Replace sequences of whitespace with a single space and trim
    std::regex ws_re(R"(\s+)");
    text = std::regex_replace(text, ws_re, " ");
    
    // Trim
    auto start = text.begin();
    while (start != text.end() && std::isspace(*start)) start++;
    auto end = text.end();
    do { end--; } while (std::distance(start, end) > 0 && std::isspace(*end));
    
    return std::string(start, end + 1);
}

std::vector<int> SimpleTokenizer::encode(const std::string& text) {
    std::vector<int> bpe_tokens;
    std::string cleaned_text = whitespace_clean(text);
    std::transform(cleaned_text.begin(), cleaned_text.end(), cleaned_text.begin(), ::tolower);
    
    // Split by regex
    std::sregex_iterator next(cleaned_text.begin(), cleaned_text.end(), pat_);
    std::sregex_iterator end;
    
    while (next != end) {
        std::string token = next->str();
        next++;
        
        // Encode to bytes mapped string
        std::string encoded_token;
        for (char c : token) {
            uint8_t byte = (uint8_t)c;
            encoded_token += byte_encoder_[byte];
        }
        
        std::string bpe_str = bpe(encoded_token);
        
        // Split bpe_str by space
        std::stringstream ss(bpe_str);
        std::string sub_token;
        while (ss >> sub_token) {
            if (encoder_.count(sub_token)) {
                bpe_tokens.push_back(encoder_[sub_token]);
            }
        }
    }
    return bpe_tokens;
}

std::string SimpleTokenizer::decode(const std::vector<int>& tokens) {
    std::string text;
    for (int t : tokens) {
        if (decoder_.count(t)) {
            text += decoder_[t];
        }
    }
    // TODO: Inverse byte encoding and </w> handling if needed for perfect reconstruction
    return text;
}

std::vector<int> SimpleTokenizer::operator()(const std::string& text, int context_length) {
    std::vector<int> tokens;
    tokens.push_back(sot_token_);
    std::vector<int> encoded = encode(text);
    tokens.insert(tokens.end(), encoded.begin(), encoded.end());
    tokens.push_back(eot_token_);
    
    if ((int)tokens.size() > context_length) {
        tokens.resize(context_length);
        tokens.back() = eot_token_;
    } else {
        while ((int)tokens.size() < context_length) {
            tokens.push_back(0);
        }
    }
    return tokens;
}

std::vector<std::vector<int>> SimpleTokenizer::operator()(const std::vector<std::string>& texts, int context_length) {
    std::vector<std::vector<int>> batch_tokens;
    for (const auto& t : texts) {
        batch_tokens.push_back((*this)(t, context_length));
    }
    return batch_tokens;
}

} // namespace yoloe_infer
