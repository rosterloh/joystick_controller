# Point to root directory.
set(REPO_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "Repo root directory")

# This is needed so that custom driver classes using system calls are taken into
# account
zephyr_syscall_include_directories(include)

zephyr_include_directories(include)

add_subdirectory(drivers)
# add_subdirectory(lib)