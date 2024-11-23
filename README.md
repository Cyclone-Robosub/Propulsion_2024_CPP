# Propulsion 2024

## Developer Resources

---

## Using Git(Hub)

### Cloning the repo
1. Install Git. This varies from system to system.
2. Open the terminal and `cd` into the directory that you want to clone the project into
3. Run the command `git clone https://github.com/Cyclone-Robosub/Propulsion_2024_CPP.git`

### Pushing Code
1. **Make sure the unit tests pass!** If your code isn't yet complete, consider making a new test branch.
    - See **"Making a New Branch"**.
2. In the terminal, navigate to the project root.
3. Run the command `git add {args}`, where `{args}` is every file that you changed and wish to push to the repo.
    - For example, if you changed the files `Command.h`, `Command_Interpreter.cpp`, and `Command_Interpreter.h`, your
   command could look like:
   ```bash
    git add Command.h Command_Interpreter.* 
   ```
4. Type a commit title on the first line in the text editor window that pops up. Keep this concise but meaningful.
5. Type a longer commit description two lines below this (hit enter twice) with more information on the commit, if needed.
6. Close the text editor.
   - Note: if you want to cancel the commit, delete your commit notes and then close the text editor.
7. Run the command `git push`
   - You will need to enter credentials here. If you don't have a Personal Access Token (PAT) configured, you will probably
   need to do so. You want a "classic" token. When prompted to authenticate, you'll type your username as your GitHub
   username and your password as your PAT (**not** your password!). More information here:
   https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens

### Pulling Code
1. In the terminal, navigate to the project directory.
2. Run the command `git pull`
   - Optionally, you can first run `git fetch` and confirm that you're sure you want to make the changes before pulling them.

### Making a New Branch
1. Pick a name for your branch.
2. In the terminal, navigate to the project directory.
3. Run the command `git branch {name}`, where `{name}` is the name of the branch.

### Switching Branches
1. In the terminal, navigate to the project directory.
2. Use the `switch {name}` command to switch between different branches.
   - For example, to switch to the branch "GTest", type `switch GTest`. 
   - Reminder: the default branch is called "master".

## Navigating the Project

### Project Structure
1. Code that we write lives in `/lib`.
2. Code that we write for unit testing (to make sure our code is correct) lives in `/testing`.
3. Data that we use for calculations lives in `/data`.
4. Other libraries that we use (like eigen) have their own directories as well.
5. You'll make your own `build/` folder (see **"Using Cmake to Build Code"**)
6. `CMakeLists.txt` and `README.md` (this file!) live on the same level as the directories listed above.

### Basic Terminal Commands
- `ls` or `dir`: These are for Unix (Linux or macOS) and Windows respectively. These list all the files and folders in 
your current location, excluding hidden files.
- `cd {path}`: This means to **c**hange **d**irectory to the directory specified in `{path}`. Path can be either a 
"relative" path or an "absolute" path.
  - A relative path is a path defined from your current location. If you're used to a graphical file explorer, you can
  think of this as all the folders and files that you can see from within whatever folder you're currently in.
  - An absolute path is a path defined from the "root" (start) of the file system. If you're on a Unix system, this
  looks like `/`. If you're on Windows, you'll recognise it as `C:\ `.
  - There are a few "special" files. `.` means "the current directory" and `..` means "the directory one level above".
    - For example, if you're in `/home/me/Documents/Cyclone_Robosub`, then `.` is  `/home/me/Documents/Cyclone_Robosub`
    and `..` is `/home/me/Documents`.
    - You can `cd` to these just like any other directory.
- For more help with terminal commands, check the Internet. Make sure that you specify your OS: the commands can be very
different depending on whether you're on a Unix system or a Windows machine.
  - With C/C++ programming, a Unix system is generally preferred. If you're using Windows, consider configuring WSL or a
  Linux virtual machine.

## Running 

### Using CMake to Build Code

1. Install CMake. This varies from system to system.
2. Open the terminal and `cd` to `/Propulsion_2024`. If you run `ls` here, you should see folders with names `lib/`, `testing/`, etc.
3. Run `cmake -B build`
4. Run `cd build`
5. Run `make`. You should get two executables: `Propulsion_2024`, and `run_tests`.
6. Run with `./run_tests` or `./Propulsion_2024`.

### Running Unit Tests
> Before you push code to the repo, you should make sure that you pass all the unit tests. Here's how:
1. Follow the instructions above to build the code (Using CMake to build code)
2. Run with `./run_tests`
3. Confirm that all tests pass (are green). If there are any failed (red) tests, check why they're failing and get them fixed!

### Making Unit Tests
You should write your tests in the `testing/` folder, in the testing file that corresponds with the file or class that
you're testing.

The format for a unit test is:
```C++
TEST(Test_Category, Test_Name) {
    //Test here
}
```
`Test_Category` refers to what component you're testing. For example, if you're testing `Command_Interpreter.cpp`, then
the test should be called `CommandInterpreterTest`. `Test_Name` refers to the name of this specific test: for example, if
you are testing that you can execute a command, call it `ExecuteCommand`.

Inside the test, you can add assertions to check that the code is performing as expected. These will be evaluated to determine
if the test passes when it is run. These look like `ASSERT_EQ(val1, val2)` (determines whether `val1` is equal to `val1`),
`ASSERT_TRUE(val)` (determines whether `val` evaluates to `true` or not), etc. If no assertions are added, the test will
pass if it doesn't crash, and fail if it does (without checking if the code is working correctly or not).

To run these tests, see **"Running Unit Tests"**

---

Code by Propulsion subteam of UC Davis Cyclone Robosub. README by William Barber.