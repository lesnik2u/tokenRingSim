#pragma once
#include <string>
#include <format>
#include <memory>

class DataItem {
private:
    std::shared_ptr<const std::string> key;
    std::shared_ptr<const std::string> value;
    int hash;
    bool isReplica;

    auto calculateHash(const std::string& str) -> int {
        std::hash<std::string> hasher;
        size_t h = hasher(str);
        return static_cast<int>(h % 360); // Hash to 0-360 range (degrees)
    }

public:
    // Efficient constructor sharing existing string storage
    DataItem(std::shared_ptr<const std::string> key, std::shared_ptr<const std::string> value, bool isReplica = false)
        : key(std::move(key)), value(std::move(value)), isReplica(isReplica) {
        hash = calculateHash(*this->key);
    }

    // Convenience constructor for fresh strings
    DataItem(std::string key, std::string value, bool isReplica = false)
        : key(std::make_shared<std::string>(std::move(key))), 
          value(std::make_shared<std::string>(std::move(value))), 
          isReplica(isReplica) {
        hash = calculateHash(*this->key);
    }

    auto getKey() const -> const std::string& { return *key; }
    auto getValue() const -> const std::string& { return *value; }
    
    // Access to shared pointers for propagation
    auto getKeyPtr() const -> std::shared_ptr<const std::string> { return key; }
    auto getValuePtr() const -> std::shared_ptr<const std::string> { return value; }

    auto getHash() const -> int { return hash; }
    auto getIsReplica() const -> bool { return isReplica; }
    auto setIsReplica(bool replica) -> void { isReplica = replica; }

    auto toString() const -> std::string {
        return std::format("Data[key={}, hash={}°, value={}, replica={}]", *key, hash, *value, isReplica);
    }
};
