#include <yaml_generator.h>

int main(void)
{
	RoCKInYAML log("salida.yaml");
	
	log.logPose2D(1,2,3)

	log.logRGB(&img[0], 3,3);

	
	return 0;
}
