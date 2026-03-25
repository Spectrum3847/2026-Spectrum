# Project Lombok

We use [Project Lombok](https://projectlombok.org/) annotations to simplify our Java code by automatically generating boilerplate code. This helps reduce verbosity and improves code readability.

## Lombok Annotations (@Getter & @Setter)

Lombok allows us to quickly add Getter and Setter methods to variables using annotations.

*   **`@Getter`**: Automatically generates a public getter method for the annotated field.
*   **`@Setter`**: Automatically generates a public setter method for the annotated field.

### Example

Instead of writing this:

```java
public class MyClass {
    private String name;
    private int id;

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }
}
```

With Lombok, you can write:

```java
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class MyClass {
    private String name;
    private int id;
}
```

Lombok automatically generates the `getName()`, `setName()`, `getId()`, and `setId()` methods during compilation.

## Builder Pattern

Our Lombok settings are configured such that all `@Setter` methods return the object itself (`this`). This enables them to work effectively as part of a **Builder Pattern**.

A Builder Pattern is useful for constructing complex objects step-by-step, especially when an object can have many optional parameters.

### Example (Conceptual with our Lombok setup)

```java
// Assuming MyClass has @Setter and returns 'this' from setters
MyClass obj = new MyClass()
    .setName("New Name")
    .setId(123);
```

This chaining of setter calls makes object creation more readable and less error-prone compared to constructors with many arguments.

## Understanding Lombok

While Lombok simplifies code, it's important to understand what it's doing behind the scenes. The generated methods are present in the compiled `.class` files, even though they are not explicitly written in the `.java` source files. This is handled by a "annotation processor" during compilation.
