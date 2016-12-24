BOOST_INCLUDES=

g++ -std=c++11 re-macro-3-vector-params.cpp -I $BOOST_INCLUDES -o re-macro

# stop at preproc
#g++ -std=c++11 re-macro-3-vector-params.cpp -I $BOOST_INCLUDES -E | tail -50
