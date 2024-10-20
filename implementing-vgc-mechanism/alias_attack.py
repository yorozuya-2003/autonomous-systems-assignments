import random
from copy import deepcopy
from vcg import VCG


def generate_bidders(auction_items, total_bidders, max_bid_value=5):
    bidder_names = [f'bidder_{i+1}' for i in range(total_bidders)]

    def generate_bids_for_bidder():
        items_already_bid_on = set()
        bids_by_bidder = {}
        for bid_num in range(random.randint(1, 5)):
            available_items_for_bid = list(set(auction_items) - items_already_bid_on)
            if not available_items_for_bid:
                break
            selected_items_for_bid = random.sample(available_items_for_bid, random.randint(1, len(available_items_for_bid)))
            items_already_bid_on.update(selected_items_for_bid)
            bids_by_bidder[f'bid_{bid_num+1}'] = {'value': random.randint(1, max_bid_value), 'items': selected_items_for_bid}
        return bids_by_bidder

    bids_by_all_bidders = {bidder: generate_bids_for_bidder() for bidder in bidder_names}

    if not any(len(bid['items']) == len(auction_items) for bids in bids_by_all_bidders.values() for bid in bids.values()):
        # recursive call
        return generate_bidders(auction_items, total_bidders)

    return bids_by_all_bidders


def vcg_alias_attack(max_bid_value=5):
    random.seed(0)
    
    for _ in range(10000):
        bidder_count = random.randint(5, 10)
        auction_items = ['top_banner', 'side_banner']
        bidders_data = generate_bidders(auction_items, bidder_count, max_bid_value)

        # running initial auction
        initial_vcg = VCG(auction_items, bidders_data)
        random_bidder = random.choice(list(bidders_data.keys()))
        original_cost_for_winner = initial_vcg.social_cost[random_bidder]
        original_won_items = {item for bidder, bid in initial_vcg.result if bidder == random_bidder 
                              for item in bidders_data[bidder][bid]["items"]}
        
        # creating an alias for the selected bidder
        alias_bidder = f"{random_bidder}_alias"
        bidders_data[alias_bidder] = deepcopy(bidders_data[random_bidder])
        bidders_data[alias_bidder]["bid_1"]["value"] = random.randint(1, max_bid_value)

        # running auction with the alias added
        alias_vcg = VCG(auction_items, bidders_data)
        new_cost_for_winner = alias_vcg.social_cost[random_bidder] + alias_vcg.social_cost.get(alias_bidder, 0)
        new_won_items = {item for bidder, bid in alias_vcg.result if bidder == random_bidder or bidder == alias_bidder 
                         for item in bidders_data[bidder][bid]["items"]}

        # checking if the alias manipulation yields a better outcome
        if original_won_items.issubset(new_won_items) and new_cost_for_winner < original_cost_for_winner:
            print(bidders_data)
            print(f"alias manipulation detected for bidder {random_bidder}.")
            print(f"original cost: {original_cost_for_winner}, new cost with alias: {new_cost_for_winner}")
            print(f"items won originally: {original_won_items}, items won with alias: {new_won_items}")
            break


if __name__ == '__main__':
    vcg_alias_attack()
