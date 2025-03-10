project('c_examples', 'c', default_options: ['c_std=gnu99'])

aravis_dep = dependency('aravis-0.8', version: '>=0.7')
mescan_dep = dependency('mescan', version: '>=1.0.1')

executable('get_profiles_callback', 'get_profiles_callback/get_profiles_callback.c',
                                        dependencies: [aravis_dep, mescan_dep],
										 )

executable('get_profiles_poll', 'get_profiles_poll/get_profiles_poll.c',
                                        dependencies: [aravis_dep, mescan_dep],
										 )

executable('partial_profile', 'partial_profile/partial_profile.c',
                                        dependencies: [aravis_dep, mescan_dep],
										 )

executable('set_ip_address', 'set_ip_address/set_ip_address.c',
                                        dependencies: [aravis_dep, mescan_dep],
                                         )
