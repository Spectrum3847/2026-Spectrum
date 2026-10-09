# Project Lombok

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

[Lombok](https://projectlombok.org/) is an annotation processor that generates boilerplate at compile time, so we do not hand-write getters, setters, or constructors. The plugin is applied from the `io.freefair.lombok` block in [`build.gradle`](../../build.gradle), which is where the pinned version lives. Read it there rather than here, so this page never disagrees with the build. This page is about which annotations we use and which we do not.

## The two you will see most

`@Getter` and `@Setter` sit on the `*Config` inner classes that hold each subsystem's tunable values. The pattern, in shape:

```java
@Getter @Setter private double someGain = 0.5;
@Getter private final double someLimit = 12.0;
```

Compiled output is `getSomeGain()` / `setSomeGain(double)` and friends, exactly as if you had written them. The point is that each line of a config is one field, one default, and no accessor clutter, so a reader scanning a config sees the numbers rather than the plumbing.

If you forget the annotation and call the accessor from somewhere else, the compile error names a method that does not exist. That is the normal failure mode and it is a two-second fix.

## Chained setters

Lombok's default setter returns `void`. We sometimes want it to return `this` so a config assembled at a call site reads as one expression:

```java
config.setName("limelight").setAttached(true);
```

`@Accessors(chain = true)` on the class is what produces that. The annotation is per class, so adding it to one config does nothing for any other. Without it the setters return `void`, the chain produces a compile error, and the error message ("cannot invoke on void") is easy to misread as a missing method.

`Limelight.LimelightConfig` is the example in this codebase. Mechanism configs deliberately do *not* chain: a mechanism's config is immutable once constructed, and a settable gear ratio is a bug waiting to happen. See [Class Generation](class-generation.md).

## What we use beyond those

`@RequiredArgsConstructor` appears on a couple of constants holders and value objects where a constructor over the final fields is exactly what is wanted.

That is close to the whole list. `@AllArgsConstructor`, `@NoArgsConstructor`, and `@Builder` are not used here; the chained-setter pattern covers the case a builder would, and writing the constructor out is shorter for the small value types we have.

What we do not use, and why:

* `@Data`: it generates `equals` and `hashCode` over every field, which interacts badly with mutable configs and is rarely the contract we want.
* `@EqualsAndHashCode` and `@ToString`: when we need them we would rather read them written out.
* `@SneakyThrows`: see [Exception Handling](exception-handling.md).
* `@Synchronized`: the parts of robot code that matter are single-threaded.

## How it works, briefly

Lombok hooks into the Java compiler as an annotation processor. When `compileJava` runs, it emits the generated methods straight into the `.class` files. There is no generated source in the repo, and the `.java` files genuinely do not contain the getters.

Two consequences:

1. Your IDE needs the Lombok plugin to see the generated methods. Without it, every Lombok-annotated class looks like it is missing methods even though the build succeeds. In VSCode that is the *Lombok Annotations Support* extension; install it before assuming the codebase is broken.
2. Do not try to commit generated code. It does not exist in source, only at compile time.

## When not to use Lombok

* When you want validation in a setter, such as clamping a gain to a range. The annotation generates a plain assignment; hand-write that one.
* On `static` fields. Lombok still generates static accessors, and they read badly.
* On a single-use POJO. A record is cleaner when the class has three fields and one use site.

## See also

* [Build Tools](../tools/build-tools.md) for the plugin wiring and the VSCode extensions.
* [Class Generation](class-generation.md) for the config pattern these annotations serve.
* Lombok's [annotation catalog](https://projectlombok.org/features/) for what each annotation does.
