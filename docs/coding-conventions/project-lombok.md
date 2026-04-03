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

## Chained Setters with `@Accessors(chain = true)`

By default, Lombok's `@Setter` methods are generated as `void` methods and do **not** return `this`. Setter chaining is **not** available unless you explicitly annotate the class with `@Accessors(chain = true)`.

Chaining example using `@Accessors(chain = true)`:

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

With `@Accessors(chain = true)`, setters return `this` so they can be chained:

```java
MyClass obj = new MyClass()
    .setName("New Name")
    .setId(123);
```

Our codebase only uses chained setters in classes that are explicitly annotated with `@Accessors(chain = true)` (for example, [src/main/java/frc/spectrumLib/vision/Limelight.java](../../src/main/java/frc/spectrumLib/vision/Limelight.java#L20)). Always add `@Accessors(chain = true)` to the class when you need chaining — do not assume it is available by default.

## Understanding Lombok

While Lombok simplifies code, it's important to understand what it's doing behind the scenes. The generated methods are present in the compiled `.class` files, even though they are not explicitly written in the `.java` source files. This is handled by a "annotation processor" during compilation.
