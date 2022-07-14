
#include <stdio.h>
#include <cs50.h>
int main() {
    int two_pound_coins = 0;
    int one_pound_coins = 0;
    int fifty_pence_coins = 0;
    int twenty_pence_coins = 0;
    int ten_pence_coins = 0;
    int five_pence_coins = 0;
    int two_pence_coins = 0;
    int one_pence_coins = 0;
    
    float change_owed = get_float("Enter how much change you owe: ");
    while (change_owed > 2.00)
    {
        change_owed -=2.00;
        two_pound_coins +=1;
    }
    while (change_owed > 1.00)
    {
        change_owed = change_owed -1.00;
        one_pound_coins = one_pound_coins +1;
    }
    while (change_owed > 0.50)
    {
        change_owed -=0.50;
        fifty_pence_coins +=1;
    }    
    while (change_owed > 0.20)
    {
        change_owed -=0.20;
        twenty_pence_coins +=1;
    }  
    while (change_owed > 0.10)
    {
        change_owed -=0.10;
        ten_pence_coins +=1;
    }  
    while (change_owed > 0.05)
    {
        change_owed -=0.05;
        five_pence_coins +=1;
    }  
    while (change_owed > 0.02)
    {
        change_owed -=0.02;
        two_pence_coins +=1;
    } 
    while (change_owed > 0.01)
    {
        change_owed -=0.01;
        one_pence_coins +=1;
    }   
    printf("You owe %i two pound coins\n",two_pound_coins);
    printf("You owe %i one pound coins\n",one_pound_coins);
    printf("You owe %i fifty pence coins\n",fifty_pence_coins);
    printf("You owe %i twenty pence coins\n",twenty_pence_coins);
    printf("You owe %i ten pence coins\n",ten_pence_coins);
    printf("You owe %i five pence coins\n",five_pence_coins);
    printf("You owe %i two pence coins\n",two_pence_coins);
    printf("You owe %i one pence coins\n",one_pence_coins);    

}
