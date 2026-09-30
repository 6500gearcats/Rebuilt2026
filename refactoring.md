# Refactoring Instructions
(By Bennett Petzold, ask him for clarifications)

Refactoring is a standard software engineering pattern that will reduce the
likelihood of bugs in competition, ensure the entire programming team is more
familiar with the codebase, and improve the speed at which bugs can be fixed in
competition.
Please see https://en.wikipedia.org/wiki/Code\_refactoring for an overview.
I do not recommend the use of any of the listed tools for this refactoring.
We will be using physical tests in place of unit tests.

All programming members can independently work on this outside the build space.
I've laid the process out in chronological order below.

## Issues
All refactoring targets start in the GitHub issues tab.
They are given the `refactor` label and describe code problems in `main`.
Team members can look over these items and assign it to themselves and/or
discuss inside the Issue.
I will be populating a few Issues, but anybody is free to add to the list.

## Make a Branch
`git branch` on the command line, search for `branch` in your VSCode palette
or find it through the Git sidebar.

## Simple Refactoring Process
Look over the code and build an understanding of what it does.
Ask the original author if you do not follow what any given part of the code is
doing.
Once you have a grasp on it, you can fix it up.
I have a noncomprehensive list of strategies below.

### Put Each Type of Change in One Commit
Discrete commits making a clear type of change (e.g. "Move magic numbers to
constants") make review easier.
It is okay if you have unclear commits, but discrete is generally best.

### Eliminate Magic Numbers
Turn all [magic numbers](https://en.wikipedia.org/wiki/Magic_number_(programming)) into constants.
In Java, those are typically created as `static final VAR_NAME = VALUE;`.
`final` prevents reassigning the value (making it constant).
`static` means that there is only one instance.
The name is conventionally in a fully capitalized snake case to help quickly
recognizing constants.
If a value in a class relies on only being assigned in the constructor and only
once, it is also a good idea to mark it as `final varName;` to prevent
overwrites during functions called in the class.

### Use Good Names
Make sure all variable names make sense.
Ideally you can identify what a variable means without looking at the
assignment.
Function names should also be as clear as possible.
Use [javadoc](https://en.wikipedia.org/wiki/Javadoc) to add description to any
function where the name does not describe the behavior well enough.

### Compartmentalize Logic
Move any self-contained blocks of code into an appropriately named function.
If possible, make them `static` functions that take explicit arguments instead
of being object functions.
This ensures they cannot do anything unexpected to the object fields.
Smaller and more often static functions are also faster to read and much easier
to unit test.
If you cannot succinctly describe what a function does, it probably needs the
internal logic split into subfunctions.

### Deduplicate Logic
Never write something twice unless it makes the code contorted.
Calling a function in each arm of a branch (if or match statement) can usually
be replaced with using variables for the arguments, assigning those in the
branch, and using a single function call.

## Fix Bugs
If you find anything that does not behave as intended, fix it.

## Design Test(s)
Come up with a routine on the robot that relies on the code you're refactoring.
This routine will be run with the original code and your changed code.
It should be designed so that the team will catch any mistakes that result in
changed behavior.
If the code was not buggy, the two executions should be identical.
If you fixed a bug, the first execution hopefully demonstrates the bug so the
second execution can show the fix.
Write out a few routines if one isn't enough to test the full functionality.

## Make a PR
Go to the Pull Request tab and make one using your branch.
[Link the issue you assigned yourself](https://docs.github.com/en/issues/tracking-your-work-with-issues/using-issues/linking-a-pull-request-to-an-issue).
Add Bennett Petzold and the original author as reviewers.
Summarize the changes you made in the description (do not go line-by-line, we
can read the code changed tab) and put in your tests.
Once it has approvals from code review, and the tests are successfully run on
the robot, the pull request will be merged.

## Repeat
Or take a break.
Congrats on making it through!
This is a lot of work, but it is a critical part of quality software
development.
