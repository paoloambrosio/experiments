#include <iostream>
#include <sstream>
#include <vector>
#include <boost/preprocessor.hpp>
#include <boost/assign/list_of.hpp>

using namespace std;
using namespace boost::assign;

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



/*
 * Defining transformers for data tables so that
 * they can be put into the parameter list
 */

struct DataTable {
    std::string content;
    DataTable() {}
    DataTable(const DataTable &other): content(other.content) {}
};

// of course not a real implementation but it's just to show how
// the template specialisation could work
template<>
DataTable fromString(const string& s, DataTable *unused) {
    DataTable dt;
    dt.content = fromString(s, &dt.content);
    return dt;
}



/*
 * DTO transformation
 */

struct DTO {
    int content;
    DTO() {}
//    DTO(const DTO &other): content(other.content) {}
};

std::istream &operator>>(std::istream &is, DTO &dto)
{
    is >> dto.content;
    return is;
}



#if BOOST_PP_VARIADICS == 0
#error Variadic macros not supported (use old style)
#endif

#define PARAM_PREFIX tp

#define DECLARE_VAR(r, data, i, elem) elem, *BOOST_PP_CAT(PARAM_PREFIX, i);
#define DECLARE_VARS_SEQ(varseq) BOOST_PP_SEQ_FOR_EACH_I(DECLARE_VAR, _, varseq)
#define DECLARE_VARS(...) DECLARE_VARS_SEQ(BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))

#define CONVERT_PARAM(z, i, data) fromString(params[i], BOOST_PP_CAT(PARAM_PREFIX, i))
#define CONVERT_PARAMS(...) BOOST_PP_ENUM(BOOST_PP_VARIADIC_SIZE(__VA_ARGS__), CONVERT_PARAM, _)

#define STEP(regex, ...)                                            \
struct Testractor {                                                 \
                                                                    \
  DECLARE_VARS(__VA_ARGS__)                                         \
                                                                    \
  Testractor() {}                                                   \
                                                                    \
  const string outer(const vector<string> &params) {                \
    return inner(CONVERT_PARAMS(__VA_ARGS__));                      \
  }                                                                 \
                                                                    \
  const string inner(__VA_ARGS__);                                  \
};                                                                  \
const string Testractor::inner(__VA_ARGS__)



STEP("some regex", int a, long b, DTO dto, string c, DataTable dt) {
  stringstream ss;
  ss << a + b + dto.content << c << dt.content;
  return ss.str();
}

int main() {
  Testractor t;
  const vector<string> params = list_of("39")("1")("2")("=")("42");
  cout << t.outer(params) << endl;
}
