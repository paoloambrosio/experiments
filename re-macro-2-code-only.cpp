#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

template<typename T>
T fromString(const string& s, T *usedToInferTemplateParameter = NULL) {
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

  // [BC] For Backward Compatibility

  int a, *tp1; // Pointer to avoid object initialisation or copies
  long b, *tp2;
  string c, *tp3;
  vector<string> pv; // [BC]

  Testractor() {}

  const string outer(
      const string &sp1,
      const string &sp2,
      const string &sp3,
      const string &sp4
    ) {
    pv.push_back(sp4); // [BC]
    return inner(
      fromString(sp1, tp1),
      fromString(sp2, tp2),
      fromString(sp3, tp3)
    );
  }

  const string inner(const int a, const long b, const string c) {
    int d = fromString<int>(pv.back()); // [BC]
    stringstream ss;
    ss << a + b << c << " != " << d;
    return ss.str();
  }
};

int main() {
  Testractor t;
  cout << t.outer("40","2"," == 42", "22") << endl;
}
