#include <iostream>
#include <sstream>

using namespace std;

template<typename T>
T fromString(const string& s, T *unused) {
    istringstream stream(s);
    T t;
    stream >> t;
    if (stream.fail()) {
        throw invalid_argument("Cannot convert parameter");
    }
    return t;
}

template<>
inline string fromString(const string& s, string *unused) {
    return s;
}


struct Testractor {

  int a, *tp1;
  long b, *tp2;
  string c, *tp3;

  Testractor() {}

  const string outer(
      const string &sp1,
      const string &sp2,
      const string &sp3
    ) {
    return inner(
      fromString(sp1, tp1),
      fromString(sp2, tp2),
      fromString(sp3, tp3)
    );
  }

  const string inner(const int a, const long b, const string c) {
    stringstream ss;
    ss << a + b << c;
    return ss.str();
  }
};

int main() {
  Testractor t;
  cout << t.outer("40","2","=42") << endl;
}
