// Utility functions

#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <algorithm>
#include <exception>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <chrono>
using std::vector;
using std::string;
using std::stringstream;
using std::setprecision;

enum Mode {DFS, BFS, DepthCont, DepthCont2, DepthCont3, DepthCont5, DepthCont10, numFix3, numFix5, numFix10, numFix15, numFix20, WeightCont};
enum tbMode { FIFO, LIFO, ARB, MinParent };

/*** Constants and Types ***/
const double Tolerance = 0.000000001;
const double Infinity = std::numeric_limits<double>::max();
const double nInfinity = std::numeric_limits<double>::min();
const int MaxInt = std::numeric_limits<int>::max();
const int MinInt = std::numeric_limits<int>::min();

/*** Error handling ***/
class Error : public std::exception
{
public:
	Error(const char* file, const int line)
	{
		_msg << setprecision(10) << "ERROR (" << file << ":" << line << "): ";
	}
	Error(const Error& e) { _msg << e._msg.str(); }
	virtual const char* what() const throw() { return (_msg.str() + "\n").c_str(); }
	~Error() throw () {}

	Error& operator<<(const char* s) { _msg << s; return *this; }
	Error& operator<<(const string s) { _msg << s; return *this; }
	Error& operator<<(const int i) { _msg << i; return *this; }
	Error& operator<<(const double f) { _msg << f; return *this; }
	Error& operator<<(const bool b) { _msg << (b ? "true" : "false"); return *this; }
	Error& operator<<(const void* p) { _msg << p; return *this; }

private:
	stringstream _msg;
};

#define ERROR Error(__FILE__, __LINE__)

/*** Miscellaneous utility functions ***/
template <typename ContainerT, typename U>
bool contains(const ContainerT& con, const U& el)
{
	typedef typename ContainerT::value_type ElementT;
	return find(con.begin(), con.end(), static_cast<ElementT>(el)) != con.end();
}

template <typename T>
void sort(T& con) { sort(con.begin(), con.end()); }

template <typename T>
void reverse(T& con) { reverse(con.begin(), con.end()); }

template <typename T>
void append(T& c1, const T& c2) { c1.insert(c1.end(), c2.begin(), c2.end()); }

template <typename U>
int vec_index(const vector<U>& vec, const U& el)
{
	return find(vec.begin(), vec.end(), el) - vec.begin();
}


#endif // UTIL_H