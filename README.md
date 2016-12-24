This is to test a new way of defining step registration macros for
Cucumber-CPP, taking advantage of variadic macros.

```
GIVEN("some regex", int a, long b, DTO dto, string c, DataTable dt) {
  // ...
}
```
