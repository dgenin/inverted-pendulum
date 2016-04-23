int _write(int fd, const void *buf, int count) {
  return -1;
}

int _close(int fd) {
  return -1;
}

int _lseek(int fd, int offset, int whence) {
  return -1;
}

int _read(int fd, void *buf, int count) {
  return -1;
}

void *_sbrk(int increment) {
  return 0; //NULL;
}

void __aeabi_unwind_cpp_pr0(void)
{
}
