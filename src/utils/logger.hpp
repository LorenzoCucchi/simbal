#include <cstddef>
#include <fstream>
#include <iostream>
#include <ostream>
#include <queue>
#include <string>
#include <utility>
#include <vector>

template <typename MatrixType>
class LoggerState {
 public:
  LoggerState(size_t maxMemory, std::string fileName,
              const std::vector<std::string>& names)
      : maxMemory(maxMemory), fileName(std::move(fileName)), names(names) {}

  ~LoggerState() { file.close(); }
  LoggerState(const LoggerState&) = delete;
  auto operator=(const LoggerState&) -> LoggerState& = delete;
  LoggerState(LoggerState&&) = delete;
  auto operator=(LoggerState&&) -> LoggerState& = delete;

  void initLog() {
    file.open(fileName, std::ofstream::trunc);
    if (file.is_open()) {
      for (const auto& name : names) {
        file << name << ",";
      }
      file << std::endl;
      std::cout << "File initialized with header line.\n";
      file.flush();
    } else {
      std::cerr << "Error: Unable to open log file.\n";
    }
  }

  void addData(const MatrixType& data) {
    dataQueue.emplace(data);
    updateMemorySize(data);
    logData();
  }

  void logData() {
    if (memorySize > maxMemory) {
      writeToFile();
    }
  }

  void flushData() { writeToFile(); }

 private:
  std::queue<MatrixType> dataQueue;
  std::string fileName;
  std::ofstream file;
  std::vector<std::string> names;
  size_t maxMemory = 0.5 * 1024 * 1024 * 1024;
  size_t memorySize{};

  void updateMemorySize(const MatrixType& data) {
    memorySize += sizeof(double) * data.size();
  }

  void writeToFile() {
    while (!dataQueue.empty()) {
      const auto& vec = dataQueue.front();
      for (const auto& val : vec) {
        file << val << ',';
      }
      file << '\n';
      dataQueue.pop();
    }
    std::cout << "Data logged to file.\n";
    memorySize = 0;
    file.flush();
  }
};