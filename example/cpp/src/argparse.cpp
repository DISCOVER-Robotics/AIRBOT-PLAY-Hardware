#include <argparse/argparse.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("demo", "1.0");

  program.add_argument("-n", "--name").help("your name").default_value(std::string("world"));

  program.add_argument("-c", "--count").help("how many times to print").scan<'i', int>().default_value(1);

  program.add_argument("--verbose").help("enable verbose output").flag();

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::string name = program.get<std::string>("--name");
  int count = program.get<int>("--count");
  bool verbose = program.get<bool>("--verbose");

  if (verbose) {
    std::cout << "Verbose mode enabled!" << std::endl;
  }

  for (int i = 0; i < count; i++) {
    std::cout << "Hello, " << name << "!" << std::endl;
  }

  return 0;
}
