import random

def main():
    cards = [2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14]
    shuffled_cards = shuffle(cards)
    hands = distribute(shuffled_cards)
    tracker = {"Number of turns":0,"Number of wars":0,"Longest war":0}
    while True:
        tracker["Number of turns"] += 1   
        card1 = hands[0][0]
        card2 = hands[1][0]
        hands[0].pop(0)
        hands[1].pop(0)

        if card1 > card2:
            hands[0].append(card1)
            hands[0].append(card2)
        elif card2 > card1:
            hands[1].append(card2)
            hands[1].append(card1)
        else:
            result = war(hands)#1: player 1 ran out 2: player 2 ran out 3: player 1 won the war 4:player 2 won the war
            tracker["Number of wars"] += result[1]
            if result[1] > tracker["Longest war"]:
                tracker["Longest war"] = result[1]
            if result[0] == 1:
                print("Player 2 wins")
                print(tracker) 
                return
            elif result[0] == 2:
                print("Player 1 wins")
                print(tracker)    
                return
            elif result[0] == 3:
                hands[0].append(card1)
                hands[0].append(card2)
                for i in range(result[1]):
                    hands[0].append(hands[0][0])
                    hands[0].append(hands[1][0])
                    hands[0].pop(0)
                    hands[1].pop(0)

            elif result[0] == 4:
                hands[1].append(card1)
                hands[1].append(card2)
                for i in range(result[1]*2):
                    hands[1].append(hands[1][0])
                    hands[1].append(hands[0][0])
                    hands[1].pop(0)
                    hands[0].pop(0)        
        



        if len(hands[0]) == 0:
            print("Player 1 wins")
            print(tracker) 
            return
        elif len(hands[1]) == 0:
            print("Player 2 wins")    
            print(tracker) 
            return


def shuffle(cards):
    available_spaces = [space for space in range(len(cards))]
    shuffled_cards = []
    for _ in range(52):
        shuffled_cards.append(0)
    for card in cards:
        place = random.choice(available_spaces)
        available_spaces.remove(place)
        shuffled_cards[place] = card
    return shuffled_cards

def distribute(shuffled_cards ,players = 2):
    hands = []
    for player in range(players):
        hands.append([])
    j = 0   
    for card in shuffled_cards:
        if j%2 == 0:
            hands[0].append(card)  
        else:
            hands[1].append(card)
        j += 1         
    return hands         

def war(hands,war_count=1):
    try:
        card1 = hands[0][(war_count*2)-1]       
    except IndexError:
        return [1,war_count]   
    try:
        card2 = hands[1][(war_count*2)-1]       
    except IndexError:
        return [2,war_count]  
    if card1 > card2:
        return [3,war_count]
    elif card2 > card1:
        return [4,war_count]
    else:
        return war(hands,war_count+1)



        


# i=0
# for card in cards:
#     print(card)
#     cards.pop(0)
#     cards.append(card)
#     i += 1
#     if i == 150:
#         break

for i in range(10):
    main()

