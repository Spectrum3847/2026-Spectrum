# Project Lombok

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

[Lombok](https://projectlombok.org/) is an annotation processor that generates boilerplate at compile time — getters, setters, constructors, equals/hashCode — so we don't have to hand-write it. The plugin lives in [`build.gradle`](../../build.gradle) (`io.freefair.lombok` 9.1.0), so the wiring is already done. This page is about which annotations we actually use and where.

## The Two Annotations You'll See Most

`@Getter` and `@Setter` are everywhere. They sit on the inner `*Config` classes that hold tunable values for each subsystem:

```java
// from LauncherConfig (excerpt)
@Getter @Setter private double kP = 0.25;
@Getter @Setter private double maxRPM = 6000.0;
@Getter @Setter private int motorId = 30;
```

Compiled output gets `getKP()`, `setKP(double)`, etc. — same as if you'd written them by hand. They keep the config classes tightly scannable: each line is one field, one default, no getter/setter clutter.

If you forget the annotation and call `config.getKP()` from elsewhere, the compile error is "method `getKP()` not found." Add `@Getter` (or `@Getter @Setter`) to the field, rebuild, fixed.

## Chained Setters: `@Accessors(chain = true)`

The Lombok default is `void setKP(double)`. We *want* setters that return `this` so per-robot configs read like a builder:

```java
this.swerveConfig = new SwerveConfig()
    .setMaxSpeed(5.0)
    .setMaxAngularRate(Math.PI * 2)
    .setCanBus(canivore);
```

To get that, annotate the class with `@Accessors(chain = true)`. Example in [`frc.spectrumLib.vision.Limelight`](../../src/main/java/frc/spectrumLib/vision/Limelight.java). Without it, the setters return `void` and chaining produces compile errors that are easy to misread ("cannot invoke `setMaxSpeed` on void").

`@Accessors(chain = true)` is per-class — annotating one class doesn't affect others. If your `*Config` needs chaining and you didn't add it, the IDE will let you write `.setX(...).setY(...)` but the compile will fail.

## Other Annotations We Use

* `@RequiredArgsConstructor` — generates a constructor taking exactly the `final` fields. Used in a few `frc.spectrumLib.util` value objects.
* `@AllArgsConstructor` — like the above but for every field. Used rarely.
* `@NoArgsConstructor` — explicit no-arg constructor when other constructors are present.
* `@Builder` — exists in Lombok but we mostly don't use it; we prefer the chained-setter pattern above.

What we *don't* use:

* `@Data` — it generates `equals`/`hashCode` based on every field, which interacts badly with mutable configs and is rarely the contract we want.
* `@EqualsAndHashCode` / `@ToString` — when we need them, we'd rather see them written out.
* `@SneakyThrows` — please no; see [Exception Handling](exception-handling.md).
* `@Synchronized` — robot code is single-threaded for the parts that matter.

## How It Works (Briefly)

Lombok hooks into the Java compiler as an annotation processor. When `compileJava` runs, Lombok scans for its annotations and emits the corresponding method bytecode directly into the `.class` files. There's no source-code generation step you can see in the repo — the `.java` files genuinely don't contain the getters.

This has two implications:

1. **Your IDE needs the Lombok plugin** to see the generated methods. Without it, every Lombok-annotated class looks like it's missing methods, even though the build succeeds. In VSCode the *Lombok Annotations Support* extension handles this; install it before assuming the codebase is broken.
2. **Don't try to commit generated code.** It doesn't exist in source — only at compile time.

## When NOT to Use Lombok

* When you want validation in a setter (`setKP` clamping to a range, say). Hand-write that one — the annotation generates a plain assignment.
* On `static` fields. Lombok still generates static accessors, but they're confusing to read.
* On a single-use POJO. If the class has three fields and one use site, `@Getter @Setter` is fine, but a record (`record Foo(int x, int y)`) is cleaner.

## See Also

* [Build Tools](../tools/build-tools.md) for the Lombok plugin wiring.
* [Class Generation](class-generation.md) for the per-subsystem config pattern that uses these annotations.
* Lombok's [official docs](https://projectlombok.org/features/) for the full annotation catalog.
