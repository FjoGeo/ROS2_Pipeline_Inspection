project('llt', 'cpp',
        version: '1.0.1',
        license: 'LGPL')
incdir = include_directories('include')

headers = ['include/llt.h']
sources = ['src/llt.cpp']
aravis_dep = dependency('aravis-0.8', version: '>=0.7')
mescan_dep = dependency('mescan', version: '>=1.0.1')

install_headers(headers, subdir: 'mescan')

shared_library('llt', sources, include_directories: incdir, 
                                  dependencies: [aravis_dep, mescan_dep], 
                                  version:'1.0.1', 
                                  soversion:'1', 
                                  install: true,
                                  install_dir:'/usr/local/lib' )

pkg_mod = import('pkgconfig')
pkg_mod.generate( libraries: '-lllt',
                  subdirs: 'mescan',
                  version: '1.0.1',
                  name: 'libllt',
                  filebase: 'llt',
                  install_dir:'/usr/local/lib/pkgconfig',
                  description: 'Wrapper library for mescan and aravis' )