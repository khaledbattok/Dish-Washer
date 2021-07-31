#define main_mode 0
#define test_mode 1

int mode;
int test_mode_num;
int door,o_f;

int main()
{
    if(start)
    {
        if (mode == main_mode)

        else if (mode == test_mode)

    }
}

void test_fcn(int n)
{
    switch (n)
    {
    case 9:
        if(door && o_f)
        {
           iv=1;
           if(flow_count>500)
           if(w_l)
                iv=0
        }
        break;
    case 8:
        if(door && o_f)
        {
            dv=1;
            //3sec
            dv=0;
        }
        break;
    case 7:
        if(door && o_f)
        {
            if(w_l)
            {
                iv=0;
                wp=1;
                //30sec
                wp=0;
            }
            else
                iv=1;
        }
        break;
    case 6:
        if(door && o_f)
        {
            if(w_l)
            {
                iv=0;
                wp=1;
                heat=1;

                if(temp>60)
                    {
                        wp=0;
                        heat=0;
                        //end
                    }

            }
            else
            {
                iv=1;
                heat=0
            }
        }
        break;
    case 5:
        if(door && o_f)
        {
            dp=1;
            //30sec
            if(w_l==0)
                dp=0;

        }
        break;
    case 4:
        if(door && o_f)
        {
           iv=1;
           if(flow_count>500)
           if(w_l)
                iv=0
        }
        break;
    case 3:
        if(door && o_f)
        {
            if(w_l)
            {
                iv=0;
                wp=1;
                //10sec
                wp=0;
            }
            else
                iv=1;
        }
        break;
    case 2:
        if(door && o_f)
        {
            sv=1;
            //90sec
            sv=0;
        }

        break;
    case 1:
        if(door && o_f)
        {
            dp=1;
            //30sec
            if(w_l==0)
                dp=0;

        }
        break;
    case 0:
        بوق ممتد
        break;
    default:
        break;
    }
}