# Arrays and enums

*Audience: New programmers. Assumes you've read [Logic operators and strings](logic-operators.md).*

The examples on this page are invented, not taken from this robot's code.

## Arrays

An array holds a fixed number of values, all of the same type. Counting starts at zero, so the
first value is at index 0.

```java
double[] prices = {3.50, 8.00, 4.25, 7.00};
// prices[0] is 3.50, prices[1] is 8.00, prices[2] is 4.25, prices[3] is 7.00
```

The size is decided when you create the array and never changes. A four element array holds exactly
four values for the rest of its life. If you need a number of values you do not know yet, use an
`ArrayList` instead.

You can also create the array empty and fill it in afterward. Every value starts at `0.0` for a
`double` array, and at `null` for an array of objects.

```java
double[] prices = new double[4];   // four doubles, all 0.0

prices[0] = 3.50;
prices[1] = 8.00;
```

The `[]` goes after the type, not after the name. `double prices[]` is the old style and you will
not see it in this codebase.

### Fixed array or ArrayList

An `ArrayList` grows and shrinks as you add and remove. An array cannot, so adding one more value
means building a new, larger array and copying everything across.

```java
import java.util.ArrayList;

ArrayList<String> names = new ArrayList<>();
names.add("Ada");
names.add("Alan");
names.add("Grace");

names.size();   // 3
```

Use an array when the count is known and unlikely to change, such as a fixed set of sensor readings
or a lookup table. Use an `ArrayList` when items come and go.

### Reading every value

The enhanced `for` loop visits each value in turn without you tracking an index. This is the
clearest way to total up an array.

```java
double[] prices = {3.50, 8.00, 4.25, 7.00};

double total = 0;
for (double price : prices) {
    total += price;
}
// total is 22.75
```

Use the enhanced form when you do not need the index. When you do need it, write the plain form,
where `i` counts from 0 to one less than the length.

```java
double[] prices = {3.50, 8.00, 4.25, 7.00};

for (int i = 0; i < prices.length; i++) {
    System.out.println(i + " costs " + prices[i]);
}
```

Arrays of objects work the same way. Put the type name where the primitive type would go. See
[Classes, methods, and objects](classes-methods-objects.md) for what those objects are.

## Enums

An enum is a named list of the only values something is allowed to have. It replaces a bare number
or a magic string that only means something if you already know the code.

```java
enum Size {
    SMALL,
    MEDIUM,
    LARGE
}

Size size = Size.MEDIUM;
```

Because only the declared values exist, a `switch` on an enum can be checked by the compiler. A
typo is a build error rather than a bug you find during a match.

```java
boolean isBig = switch (size) {
    case SMALL -> false;
    case MEDIUM -> false;
    case LARGE -> true;
};
// isBig is false
```

An enum is a good fit anywhere you would otherwise be passing around a number or a string and
hoping everyone agrees what it means. See [Classes, methods, and objects](classes-methods-objects.md)
for how one usually sits inside a class.

## Two errors you will hit

**`ArrayIndexOutOfBoundsException`.** You asked for an index that does not exist. In a four element
array the valid indexes are 0 through 3, so index 4, or any negative index, throws this. It only
throws at runtime, so the compiler will not catch it for you.

```java
double[] prices = {3.50, 8.00};

System.out.println(prices[0]);   // 3.50
System.out.println(prices[2]);   // throws ArrayIndexOutOfBoundsException
```

**`NullPointerException`.** You asked a `null` to do something. A slot in a new array of objects
holds `null` until you put something in it, and calling a method on `null` throws straight away.

```java
String[] names = new String[3];

System.out.println(names[0].length());   // throws NullPointerException
```

---

*Previous: [Logic operators and strings](logic-operators.md). Next: [Loops](loops.md)*
