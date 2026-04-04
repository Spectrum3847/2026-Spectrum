# Code Style

We standardize our code style primarily on the **Android Open Source Project (AOSP) coding standards**.

## Spotless Integration

We use **Spotless** to automatically apply this style to all our classes whenever the project is built. This ensures a largely standardized code appearance, regardless of the individual author.

You can read the full AOSP style guide here, though it might not be entirely necessary for day-to-day coding: [https://source.android.com/docs/setup/contribute/code-style](https://source.android.com/docs/setup/contribute/code-style)

## Be Consistent

A core principle of code style is consistency.

> "One of the simplest rules is BE CONSISTENT. If you're editing code, take a few minutes to look at the surrounding code and determine its style. If that code uses spaces around the if clauses, you should, too. If the code comments have little boxes of stars around them, make your comments have little boxes of stars around them, too.
>
> The point of having style guidelines is to have a common vocabulary of coding, so readers can concentrate on what you're saying, rather than on how you're saying it. If the code that you add to a file looks drastically different from the existing code around it, it throws readers out of rhythm when they read it. Try to avoid this." - AOSP Code Style

## Capitalization Conventions

*   **Classes and Interfaces**: `CamelCase`, with the first letter capitalized.
    *   Example: `MyClass`, `MyInterface`
*   **Methods**: `camelCase`, with the first letter lowercase.
    *   Example: `calculateAverage()`, `getStudentName()`
*   **Variables**: `camelCase`, with the first letter lowercase.
    *   Example: `studentName`, `averageScore`
*   **Constants**: All uppercase letters, with words separated by underscores.
    *   This applies *only* to true constants that cannot possibly change (e.g., `Math.PI`).
    *   Anything that can potentially change from robot to robot or is a tuning value (e.g., `wheelBaseInches`) is considered a variable and should follow `camelCase`. This convention should not be used often.

## Acronyms

Treat acronyms as words; only capitalize the first letter of the acronym in most cases.

## No Prefixes on Variable Names

Avoid adding prefixes like `m_` or similar to variable names to denote member variables. If you encounter this in our code, it's likely inherited from an external library or team and should be renamed without the prefix.

## Indent Using 4 Spaces

Always use 4 spaces for indentation. When importing code from other sources, this is one of the common changes you might need to make to ensure consistency.
