project('cpp_examples', 'cpp')

aravis_dep = dependency('aravis-0.8', version: '>=0.7')
mescan_dep = dependency('mescan', version: '>=1.0.1')
llt_dep = dependency('llt', version: '>=1.0.1')
png_dep = dependency('libpng', version: '>=1.0.1')

executable('Calibration', 'Calibration/Calibration.cpp',
                                        dependencies: [aravis_dep, mescan_dep, llt_dep],
										 )

executable('ContainerMode', 'ContainerMode/ContainerMode.cpp',
                                        dependencies: [aravis_dep, mescan_dep, llt_dep, png_dep],
										 )

executable('GetProfilesCallback', 'GetProfilesCallback/GetProfilesCallback.cpp',
                                        dependencies: [aravis_dep, mescan_dep, llt_dep],
										 )

executable('GetProfilesPoll', 'GetProfilesPoll/GetProfilesPoll.cpp',
                                        dependencies: [aravis_dep, mescan_dep, llt_dep],
										 )

executable('LLTPeakFilter', 'LLTPeakFilter/LLTPeakFilter.cpp',
                                        dependencies: [aravis_dep, mescan_dep, llt_dep],
										 )

executable('PartialProfile', 'PartialProfile/PartialProfile.cpp',
                                        dependencies: [aravis_dep, mescan_dep, llt_dep],
										 )

executable('PartialProfile_MultiLLT', 'PartialProfileMultiLLT/PartialProfileMultiLLT.cpp',
                                        dependencies: [aravis_dep, mescan_dep, llt_dep],
										 )

executable('VideoMode', 'VideoMode/VideoMode.cpp',
                                        dependencies: [aravis_dep, mescan_dep, llt_dep],
										 )
