#ifndef ANG_TEST_INCLUDED
#define ANG_TEST_INCLUDED

// 4251: some class or template needs to have dll-interface to be used by clients
// of this class. No problem when refers to templates, clients can link successfully
// because the class instantiation (the code) is in client side (so no link at all).
// Disabling 4251 can hide a problem when this class (which is dll exported): either
// (1) expose in non-private sections or (2) use in inline functions implementations,
// types witch are not dll exported
// 4996:  MSVC has gone into full security-paranoia mode. std::copy issues this warning
// when it is used with raw pointers, because when used incorrectly, it can result in
// buffer overflows. But if code is OK, no problem. Stupid Microsoft warning.
#if defined(_MSC_VER)
#	pragma warning(disable:4251)
#	pragma warning(disable:4996)
#endif

#endif
