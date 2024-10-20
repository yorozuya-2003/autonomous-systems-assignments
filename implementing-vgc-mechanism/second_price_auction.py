from helpers import generate_bidder_mapping


def second_price_auction(bidder_mapping):    
    winner_bid = max_bid = None
    all_bids = []
    
    for bidder, bids in bidder_mapping.items():
        for bid, details in bids.items():
            bid_value = details["value"]
            all_bids.append(bid_value)
            if max_bid is None or bid_value > max_bid or (bid_value == max_bid and bidder < winner_bid[0]):
                winner_bid, max_bid = (bidder, bid), bid_value

    second_max_bid = sorted(all_bids, reverse=True)[1]
    return winner_bid, second_max_bid


if __name__ == '__main__':
    bidder_mapping = generate_bidder_mapping(csv_file_path='simple_bids.csv')
    print(second_price_auction(bidder_mapping))
