# Formatting Code

## Flow of Execution

### Java
*   Begins code execution with `main` function.
*   Files end in `.java` (e.g., `Main.java`).
*   Runs in order of program statements from top to bottom.

### Python
*   Files end in `.py`.

## Indentation

### Java
*   Indentations are mostly cosmetic; the compiler does not read whitespaces.
*   Uses braces `{}` to indicate what code is contained in what structure.

```java
void moveRobot() {
    moveForward();
}

int a = 5;
int b = 1;
if (a > b) {
    System.out.println("a greater than b");
} else {
    System.out.println("a not greater than b");
}
```

### Python
*   Uses indentation to compile code.

```python
def moveRobot():
    moveForward()

a = 5
b = 1
if a > b:
    print("a greater than b")
else:
    print("a not greater than b")
```

## Commenting

### Java
*   `// single line comment`
*   `/** Javadoc comments store comments across several lines of code */`

### Python
*   `# single line comments`
*   `''' Multiline comments store comments across several lines '''`
