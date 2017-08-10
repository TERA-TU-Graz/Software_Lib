//#include "module/common/modules_def.h"
//#include <sys/errno.h>
//#include <sys/stat.h>
//#include <sys/types.h>
//#include <sys/times.h>
//#include <sys/unistd.h>
//
//// push the current warning setting, don't forget to pop after bothersome code!
//// This lets us suppress 'unused parameter' warnings
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-parameter"
//
//#undef errno
//extern int errno;
//
//// environ
//// A pointer to a list of environment variables and their values.
//// For a minimal environment, this empty list is adequate:
//char *__env[1] = { 0 };
//char **environ = __env;
//
//int _close(int file) {
//    return -1;
//}
//
////------------------------------------------------------------------------------
//// execve
//// Transfer control to a new process.
//// Minimal implementation (for a system without processes):
////
//int _execve(char *name, char **argv, char **env) {
//    errno = ENOMEM;
//    return -1;
//}
////------------------------------------------------------------------------------
//// fork
//// Create a new process. Minimal implementation (for a system without processes)
////
//int _fork() {
//    errno = EAGAIN;
//    return -1;
//}
////------------------------------------------------------------------------------
//// fstat
//// Status of an open file. For consistency with other minimal implementations in
//// these examples, all files are regarded as character special devices.
//// The `sys/stat.h' header file required is distributed in the `include'
//// subdirectory for this C library.
////
//int _fstat(int file, struct stat *st) {
//    st->st_mode = S_IFCHR;
//    return 0;
//}
//
////------------------------------------------------------------------------------
//// getpid
//// Process-ID; this is sometimes used to generate strings unlikely to conflict
//// with other processes. Minimal implementation, for a system without processes
////
//int _getpid() {
//    return 1;
//}
//
////------------------------------------------------------------------------------
//// isatty
//// Query whether output stream is a terminal. For consistency with the other
//// minimal implementations,
////
//int _isatty(int file) {
//    switch (file){
//    case STDOUT_FILENO:
//    case STDERR_FILENO:
//    case STDIN_FILENO:
//        return 1;
//    default:
//        //errno = ENOTTY;
//        errno = EBADF;
//        return 0;
//    }
//}
//
////------------------------------------------------------------------------------
//// kill
//// Send a signal. Minimal implementation:
////
//int _kill(int pid, int sig) {
//    errno = EINVAL;
//    return (-1);
//}
//
////------------------------------------------------------------------------------
//// link
//// Establish a new name for an existing file. Minimal implementation:
////
//int _link(char *old, char *new) {
//    errno = EMLINK;
//    return -1;
//}
//
////------------------------------------------------------------------------------
//// lseek
//// Set position in a file. Minimal implementation:
////
//int _lseek(int file, int ptr, int dir) {
//    return 0;
//}
//
//
////------------------------------------------------------------------------------
//// sbrk
//// Increase program data space.
//// Malloc and related functions depend on this
////
//caddr_t _sbrk(int incr) {
//
//  extern char _bss_end; // Defined by the linker
//  static char *heap_end;
//  char *prev_heap_end;
//
//  if (heap_end == 0) {
//      heap_end = &_bss_end;
//  }
//  prev_heap_end = heap_end;
//
//  char * stack = (char*) __get_MSP();
//   if (heap_end + incr >  stack)
//   {
////         _write (STDERR_FILENO, "Heap and stack collision\n", 25);
//       errno = ENOMEM;
//       return  (caddr_t) -1;
//       //abort ();
//   }
//
//  heap_end += incr;
//  return (caddr_t) prev_heap_end;
//
//}
//
//int _stat(const char *filepath, struct stat *st) {
//    st->st_mode = S_IFCHR;
//    return 0;
//}
//
////------------------------------------------------------------------------------
//// times
//// Timing information for current process. Minimal implementation:
////
//clock_t _times(struct tms *buf) {
//    return -1;
//}
//
////------------------------------------------------------------------------------
//// unlink
//// Remove a file's directory entry. Minimal implementation:
////
//int _unlink(char *name) {
//    errno = ENOENT;
//    return -1;
//}
//
////------------------------------------------------------------------------------
//// wait
//// Wait for a child process. Minimal implementation:
////
//int _wait(int *status) {
//    errno = ECHILD;
//    return -1;
//}
//
//
//#pragma GCC diagnostic pop // Here we restore the previous warning settings
