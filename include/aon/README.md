# README File Template for Components

## Description
Here, describe your component and what it does in 1-3 sentences. It should not be a technical description of code or mechanics, only what the component adds to the robot. 

Example: The conveyor belt grabs our items and secures them in the goal carried in the back of the robot. The code can be adjusted to work for other positions if needed.

## Hardware Requirements
- Belt (1)
- Motors (2)
- Vision Sensor (1)
- Distance Sensor (1)
- Belt Spokes (7)

## Details
In this section you may include some important details of your component, this includes expected behavior, unexpected behavior, code snippets that have produced desired results, code with mistakes that should be avoided, etc. Try to keep it brief, verbose explanations help no one.

```
#include <bits/stdc++.h>

using namespace std;

int main()
{
	cout << "This code works perfecty fine" << endl;
	
	return 0;
}
```
This code has nothing wrong with it.

```
void makeMistake(const int &num)
{
	std::cout << "Lets change the number!" << endl;
	
	try {
		num += 18;
	} 
	catch {
		std::cout << "Something bad happened." << endl;
	}
}
```
This code will produce an error, but I handled it.




