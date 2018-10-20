#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define DRIVER_NAME "mlb_input"

struct milb_key_map{
	int p_no;
	int gpio_no;
	u32 event;
	char* real_key;
};
const struct milb_key_map key_map_tbl[] = {
	{2, 258, BTN_0, "AF"}, //AF			BTN_0
	{3, 259, BTN_1, "Shutter"}, //shutter		BTN_1
        {8, 264, BTN_2, "Cross_up"}, //cross up			BTN_2
        {9, 265, BTN_3, "Cross_left"}, //cross left		BTN_3
        {10, 266, BTN_4, "Cross_down"}, //cross down		BTN_4
        {11, 267, BTN_5, "Cross_right"}, //cross right		BTN_5
        {12, 268, BTN_6, "Cross_center"}, //cross center	BTN_6
        {17, 271, BTN_7, "p17 debug"},
        {31, 281, BTN_8, "p31 debug"},
        {32, 282, BTN_9, "p32 debug"},
        {33, 283, BTN_A, "p33 debug"}, 

};

struct milb_input{
  struct input_dev *input;
  struct device *dev;

  struct delayed_work work;
};

static int rotary_reported_key_no = 0;

struct rotary_key_map{
        int gpio_combination;
        u32 event;
};

const struct rotary_key_map rotary_map_tbl[] = {
	{0, KEY_0},
        {1, KEY_1},
        {2, KEY_2},
        {3, KEY_3},
        {4, KEY_4},
        {5, KEY_5},
        {6, KEY_6},
        {7, KEY_7},
        {8, KEY_8},
        {9, KEY_9},
        {10, KEY_A},
        {11, KEY_B},
        {12, KEY_C},
        {13, KEY_D},
        {14, KEY_E},
        {15, KEY_F},
};


static void report_events(struct milb_input *mlb){
	int i, size, val;
	unsigned int rotary;

	size = sizeof(key_map_tbl)/sizeof(key_map_tbl[0]);

	for(i=0; i<size; i++){
		val = gpio_get_value(key_map_tbl[i].gpio_no);
		if(val==1){
			pr_info("report key event %d(%s)\n", key_map_tbl[i].event, key_map_tbl[i].real_key);
                        input_report_key(mlb->input, key_map_tbl[i].event, 1);
		}else{
                        input_report_key(mlb->input, key_map_tbl[i].event, 0);
		}
		input_sync(mlb->input);
	}
	
	//for rotary key
	rotary = 0;
	for(i=0; i<4; i++){
		val = gpio_get_value(284+i);
		rotary |= (val<<i);
	}
	rotary = 0xf - rotary;//translate the gpio number (2^4) to rotary key number.

	if(rotary >=0 && rotary <=15){//skip 0 as default, do nothing.
		if(rotary_reported_key_no != rotary){//not yet report or the last key report is not the same.
			pr_info("will report rotary key %d \n", rotary_map_tbl[rotary].event);

			if (rotary_reported_key_no !=0 ){
				//release event for last key
				input_report_key(mlb->input, rotary_map_tbl[rotary_reported_key_no].event, 0);
                        	input_sync(mlb->input);
			}

			rotary_reported_key_no = rotary;

			if (rotary!=0){
				//report current key
				input_report_key(mlb->input, rotary_map_tbl[rotary].event, 1);
				input_sync(mlb->input);
			}
		}

	}	
}
static void input_work(struct work_struct *work)
{
        struct milb_input *mlb =
                        container_of(work, struct milb_input,
                                     work.work);
	report_events(mlb);

	schedule_delayed_work(&mlb->work, HZ);
}

static int mlb_input_probe(struct platform_device *pdev)
{
	struct milb_input *mlb_input;
	int error;
	
	pr_info("%s\n", __func__);

	mlb_input = kzalloc(sizeof(struct milb_input), GFP_KERNEL);

	mlb_input->input = input_allocate_device();
	if (!mlb_input->input){
		pr_info("cas: %s, !input return \n", __func__);
		return -ENOMEM;
	}

	mlb_input->input->name = pdev->name;
	mlb_input->input->phys = "mlb/input0";
	mlb_input->input->id.bustype = BUS_HOST;
	mlb_input->input->dev.parent = &pdev->dev;
	
        input_set_capability(mlb_input->input, EV_KEY, BTN_0);
        input_set_capability(mlb_input->input, EV_KEY, BTN_1);
        input_set_capability(mlb_input->input, EV_KEY, BTN_2);
        input_set_capability(mlb_input->input, EV_KEY, BTN_3);
        input_set_capability(mlb_input->input, EV_KEY, BTN_4);
        input_set_capability(mlb_input->input, EV_KEY, BTN_5);
        input_set_capability(mlb_input->input, EV_KEY, BTN_6);
	input_set_capability(mlb_input->input, EV_KEY, BTN_7);
	input_set_capability(mlb_input->input, EV_KEY, BTN_8);
	input_set_capability(mlb_input->input, EV_KEY, BTN_9);
	input_set_capability(mlb_input->input, EV_KEY, BTN_A);

        //input_set_capability(mlb_input->input, EV_KEY, KEY_0);
        input_set_capability(mlb_input->input, EV_KEY, KEY_1);
        input_set_capability(mlb_input->input, EV_KEY, KEY_2);
        input_set_capability(mlb_input->input, EV_KEY, KEY_3);
        input_set_capability(mlb_input->input, EV_KEY, KEY_4);
        input_set_capability(mlb_input->input, EV_KEY, KEY_5);
        input_set_capability(mlb_input->input, EV_KEY, KEY_6);
        input_set_capability(mlb_input->input, EV_KEY, KEY_7);
        input_set_capability(mlb_input->input, EV_KEY, KEY_8);
        input_set_capability(mlb_input->input, EV_KEY, KEY_9);
        input_set_capability(mlb_input->input, EV_KEY, KEY_A);
        input_set_capability(mlb_input->input, EV_KEY, KEY_B);
        input_set_capability(mlb_input->input, EV_KEY, KEY_C);
        input_set_capability(mlb_input->input, EV_KEY, KEY_D);
        input_set_capability(mlb_input->input, EV_KEY, KEY_E);
        input_set_capability(mlb_input->input, EV_KEY, KEY_F);
	

	error = input_register_device(mlb_input->input);
	if (error) {
		pr_info("cas: Unable to register input dev, error "
				"%d\n", error);
		return -1;
	}

        INIT_DELAYED_WORK(&mlb_input->work, input_work);
        schedule_delayed_work(&mlb_input->work, HZ*10);


	platform_set_drvdata(pdev, mlb_input->input);

	pr_info("%s, end\n", __func__);
	return 0;
}

static int mlb_input_remove(struct platform_device *pdev)
{
	struct input_dev *input = platform_get_drvdata(pdev);
	input_unregister_device(input);

	return 0;
}

static const struct of_device_id mlb_input_match[] = {
        { .compatible = "socionext,mlb01-input", },
        {},
};
MODULE_DEVICE_TABLE(of, mlb_input_match);

static struct platform_driver mlb_input_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mlb_input_match,
	},
	.probe	= mlb_input_probe,
	.remove	= mlb_input_remove,
};

module_platform_driver(mlb_input_driver);


MODULE_AUTHOR("Caspar Lin<caspar.lin@socionext.com>");
MODULE_DESCRIPTION("Socionext Inc. Input Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
