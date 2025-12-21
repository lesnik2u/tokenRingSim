#pragma once
#include <format>
#include <memory>
#include <string>

class DataItem {
private:
    std::shared_ptr<const std::string> key;
    std::shared_ptr<const std::string> value;
    int hash;
    bool isReplica;

    // Helper for consistent hashing
    int calculateHash(const std::string &str) {
        std::hash<std::string> hasher;
        size_t h = hasher(str);
        return static_cast<int>(h % 360);
    }

public:
    // Constructor using shared pointers
    DataItem(std::shared_ptr<const std::string> key, std::shared_ptr<const std::string> value,
             bool isReplica = false)
        : key(std::move(key)), value(std::move(value)), isReplica(isReplica) {
        hash = calculateHash(*this->key);
    }

    // Constructor creating new strings
    DataItem(std::string key, std::string value, bool isReplica = false)
        : key(std::make_shared<std::string>(std::move(key))),
          value(std::make_shared<std::string>(std::move(value))), isReplica(isReplica) {
        hash = calculateHash(*this->key);
    }

    // Accessors
    const std::string &getKey() const { return *key; }
    const std::string &getValue() const { return *value; }

    std::shared_ptr<const std::string> getKeyPtr() const { return key; }
    std::shared_ptr<const std::string> getValuePtr() const { return value; }

    int getHash() const { return hash; }
    bool getIsReplica() const { return isReplica; }
    void setIsReplica(bool replica) { isReplica = replica; }

    std::string toString() const {
        return std::format("Data[key={}, hash={}°, value={}, replica={}]", *key, hash, *value,
                           isReplica);
    }
};