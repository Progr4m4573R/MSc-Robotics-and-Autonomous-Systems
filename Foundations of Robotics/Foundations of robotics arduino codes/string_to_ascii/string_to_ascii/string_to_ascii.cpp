// string_to_ascii.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

# include <iostream>
# include <string>
# include <vector>

std::vector<int> converttoASCII(std::string s)  //used a vector to store all our ASCII values
{
	std::vector <int> vals;  //vectpr creation
	int ascChar;
	for (int i = 0; i < s.length(); i++)  //We interate through string passed and add to vectors
	{
		ascChar = s[i];
		vals.push_back(ascChar);
	}
	return vals;
}


int main()
{
	std::string toencode;
	std::cout << "Please enter in a string to encode: ";
	std::getline(std::cin, toencode);
	while (toencode.length() == 0)  //we used a for loop to prevent user from entering nothing.
	{
		std::cin.clear();
		std::cout << "Must not be empty! Try Again.\n";
		std::cout << "Please enter in a string to encode: ";
		std::getline(std::cin, toencode);
	}
	std::vector <int> asciivals = converttoASCII(toencode);
	for (int i : asciivals)  //Print out the results of the vector
	{
		std::cout << i << "\n";
	}
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
