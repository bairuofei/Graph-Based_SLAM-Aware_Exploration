define hokuyolaser ranger
(
	sensor
	( 			
		# laser-specific properties
		# factory settings for LMS200	
		# change 30 to 5
		range [ 0.0  25.0 ]
		fov 180.0
		# samples 721
		samples 901
	)

	# generic model properties
	color "blue"
	size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet	
)



