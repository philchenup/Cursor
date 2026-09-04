/**
 * @file exports.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_EXPORTS_H
#define CT_BASE_EXPORTS_H

#define BOOST_ALLOW_DEPRECATED_HEADERS

#if defined(CT_STATIC)
#define CT_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(CT_LIBRARY)
#define CT_EXPORT __declspec(dllexport)
#else
#define CT_EXPORT __declspec(dllimport)
#endif
#else
#define CT_EXPORT __attribute__((visibility("default")))
#endif

#endif  // CT_BASE_EXPORTS_H
