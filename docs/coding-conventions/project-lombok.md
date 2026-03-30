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

By default, Lombok's `@Setter` methods are generated as `void` methods and do not return `this`. To make setters chainable (fluent setters that return the object), annotate the class or fields with `@Accessors(chain = true)` or enable chaining project-wide via a `lombok.config` file.

Chaining example using `@Accessors`:

```java
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Getter
@Setter
@Accessors(chain = true)
public class MyClass {
    private String name;
    private int id;
}
```

Now you can chain setters:

```java
MyClass obj = new MyClass()
    .setName("New Name")
    .setId(123);
```

Our codebase uses `@Accessors(chain = true)` in places (for example, [src/main/java/frc/spectrumLib/vision/Limelight.java](../../src/main/java/frc/spectrumLib/vision/Limelight.java#L20)) to enable chained setters. If you prefer chaining across the whole repository, add a `lombok.config` file at the project root with:

```
lombok.accessors.chain = true
```
This makes chained setters the default without annotating every class.

## Understanding Lombok

While Lombok simplifies code, it's important to understand what it's doing behind the scenes. The generated methods are present in the compiled `.class` files, even though they are not explicitly written in the `.java` source files. This is handled by a "annotation processor" during compilation.
