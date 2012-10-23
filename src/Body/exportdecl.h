
#ifdef CNOID_EXPORT
#undef CNOID_EXPORT
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# ifdef CnoidBody_EXPORTS
#  define CNOID_EXPORT __declspec(dllexport)
# else
#  define CNOID_EXPORT __declspec(dllimport)
# endif
#else
# define CNOID_EXPORT
#endif
