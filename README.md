# Project Setup Instructions

## Overview
This repository contains three task complexity levels. Follow the instructions below to set up your environment, install dependencies, and validate the project.

## Setup Instructions

### 1. Clone the Repository
Open your terminal and run the following command:
```bash
git clone https://github.com/codewitted/Assignment_cmp3103.git
```

### 2. Navigate to the Project Directory
```bash
cd Assignment_cmp3103
```

### 3. Install Dependencies
Make sure you have the required dependencies installed:
- Python 3.x
- Pip

If you're using a virtual environment, create and activate it first:
```bash
python -m venv venv
source venv/bin/activate  # On Windows use `venv\Scripts\activate`
```

Then, install dependencies using Pip:
```bash
pip install -r requirements.txt
```

## Troubleshooting
- If you encounter issues with dependencies, ensure your Python version meets the requirements specified in `requirements.txt`.
- Check for installation errors in the terminal, and verify internet connectivity for downloading packages.

## Validation Procedures

### Basic Validation
Run the main application to ensure it starts correctly:
```bash
python main.py
```
Expected output:
```
Application started successfully!
```

### Task Complexity Levels
1. **Level 1:** Basic functionality test
   - Command: `python level1.py`
   - Expected Behavior: Outputs necessary information regarding Level 1 tasks.

2. **Level 2:** Intermediate functionality test
   - Command: `python level2.py`
   - Expected Behavior: Outputs necessary information regarding Level 2 tasks.

3. **Level 3:** Advanced functionality test
   - Command: `python level3.py`
   - Expected Behavior: Outputs necessary information regarding Level 3 tasks.

## Build Steps
### To build the project, follow these commands:
```bash
# Build step command example
make build
```
Replace with actual commands as per your project setup.

### Verifying Success
- Confirm outputs match the expected behaviors listed above.
- Check the console for success messages or errors.

### Final Notes
For any additional issues, please consult the official documentation or open an issue in the repository.