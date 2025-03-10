project('mescan', 'c', default_options: 'c_std=gnu99',
        version: '1.0.1',
        license: 'LGPL')
incdir = include_directories('include')

cc = meson.get_compiler('c')

headers = ['include/mescan.h', 'include/mescan_basic.h', 'include/mescan_adv.h', 'include/LLTDataTypes.h']
sources = ['src/mescan_basic.c', 'src/mescan_adv.c']
aravis_dep = dependency('aravis-0.8', version: '>=0.7')
libm = cc.find_library('m', required: true)

install_headers(headers, subdir: 'mescan')

libmescan = shared_library('mescan', sources, include_directories: incdir,
                                  dependencies: [aravis_dep, libm],
                                  version:'1.0.1',
                                  soversion:'1',
                                  install: true,
                                  install_dir:'/usr/local/lib' )

pkg_mod = import('pkgconfig')
pkg_mod.generate( libraries: '-lmescan',
                  subdirs: 'mescan',
                  version: '1.0.1',
                  name: 'mescan',
                  filebase: 'mescan',
                  install_dir:'/usr/local/lib/pkgconfig',
                  description: 'Library for operating MICRO-EPSILON scanCONTROL sensors' )


# test_mescan_basic = executable('test_mescan_basic', 'tests/test_mescan_basic.c', link_with: libmescan,
#										 dependencies: [aravis_dep, glib_dep],
#										 include_directories: incdir
#										 )
# test('Test mescan basic', test_mescan_basic)