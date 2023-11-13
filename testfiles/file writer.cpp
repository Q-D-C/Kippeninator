#include <iostream>
#include <fstream>

int main()
{

  // Reading the variable from the file
  std::ifstream inFile("data.txt");
  int storedValue;
  if (inFile.is_open())
  {
    inFile >> storedValue;
    inFile.close();
    std::cout << "Value read from the file: " << storedValue << std::endl;
  }
  else
  {
    std::cerr << "Unable to open the file." << std::endl;
    return 1;
  }

  // Variable you want to store
  int value = 42;

  // Writing the variable to a file
  std::ofstream outFile("data.txt");
  if (outFile.is_open())
  {
    outFile << value;
    outFile.close();
    std::cout << "Value has been stored in the file." << std::endl;
  }
  else
  {
    std::cerr << "Unable to open the file." << std::endl;
    return 1;
  }

  return 0;
}
