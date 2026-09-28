# Project Lombok

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

[Lombok](https://projectlombok.org/) is an annotation processor that generates boilerplate at compile time (getters, setters, constructors) so we don't have to hand-write it. The plugin is already declared in [`build.gradle`](../../build.gradle), so the wiring is done. This page is about which annotations we actually use and where.

## The two annotations you'll see most

`@Getter` and `@Setter` are everywhere, on the config classes that hold tunable values for each subsystem:

```java
@Getter private final double currentLimit = 80;
@Getter @Setter private double deadband = 0.05;
```

The first is the pattern for a mechanism `*Config`: the values are constants, so getter only. The second is the pattern for a config assembled at a call site, where a value gets set once and read many times. Either way each line stays one field and one default, with no getter and setter clutter around it.

A generated name follows the field name, so `deadband` gives you `getDeadband()` and `setDeadband(double)`. If you call one of those and the compiler says the method isn't found, you forgot the annotation. Add it and rebuild.

## Chained setters, and the `with*` alternative

Lombok's default setter returns `void`. To get a setter that returns `this` so a call site reads like a builder, annotate the class with `@Accessors(chain = true)`:

```java
this.someConfig = new SomeConfig().setWidth(0.2).setHeight(0.4);
```

Exactly one class in this repo carries the annotation, `Limelight.LimelightConfig`. The chaining you will actually see in `Vision.java`, though, goes through hand-written `with*` methods on that same class (`withTranslation(...)`, `withRotation(...)`), not through the Lombok setters. So the pattern this codebase really uses for a builder-shaped config is a hand-written method that assigns and returns `this`:

```java
public SomeConfig withWidth(double width) {
    this.width = width;
    return this;
}
```

Reach for that rather than `@Accessors` when you are writing a new config. The annotation's failure mode when it's missing is confusing, because the error reads as if the method does not exist rather than as a return type mismatch, and that sends people looking in the wrong place.

The annotation is per-class either way. Putting it on one config does nothing for its neighbours, and a config that needs chaining will not have it because a neighboring config does.

## Everything else

Only `@Getter`, `@Setter`, and `@Accessors` appear in `src/main/java`. Grep for `lombok.` in that tree to confirm the current set; the list changes as the code does.

What we deliberately avoid:

* `@Data`: it generates `equals` and `hashCode` from every field, which interacts badly with mutable configs and is rarely the contract we want.
* `@EqualsAndHashCode` and `@ToString`, when we need them we would rather see them written out.
* `@SneakyThrows`: see [Exception Handling](exception-handling.md).
* `@Synchronized`: robot code is single-threaded for the parts that matter.

## How it works, briefly

Lombok hooks into the Java compiler as an annotation processor. When `compileJava` runs, Lombok scans for its annotations and emits the corresponding method bytecode directly into the `.class` files. There's no source-code generation step you can see in the repo; the `.java` files genuinely don't contain the getters.

This has two implications:

1. **Your IDE needs the Lombok plugin** to see the generated methods. Without it, every Lombok-annotated class looks like it's missing methods, even though the build succeeds. In VS Code the *Lombok Annotations Support* extension handles this; install it before assuming the codebase is broken.
2. **Don't try to commit generated code.** It doesn't exist in source, only at compile time.

## When not to use Lombok

* When you want validation in a setter, such as clamping a value to a range. Hand-write that one; the annotation generates a plain assignment.
* On `static` fields. Lombok still generates static accessors, but they're confusing to read.
* On a single-use POJO. If the class has three fields and one use site, `@Getter @Setter` is fine, but a record (`record Foo(int x, int y)`) is cleaner.

## See also

* [Build Tools](../tools/build-tools.md) for the Lombok plugin wiring.
* [Class Generation](class-generation.md) for the per-subsystem config pattern that uses these annotations.
* Lombok's [official docs](https://projectlombok.org/features/) for the full annotation catalog.
