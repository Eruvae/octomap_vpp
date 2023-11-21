import numpy as np

# compute log-odds from probability:
def logodds(probability):
    return np.log(probability/(1-probability))

# compute probability from logodds:
def probability(logodds):
    return (1 - ( 1 / (1 + np.exp(logodds))))

def main():
    NE = 3

    test_odds = np.full(NE, logodds(1/NE), dtype=np.float64)

    print(test_odds)
    print(probability(test_odds))
    print("Sum lo", np.sum(test_odds))
    print("Sum", np.sum(probability(test_odds)))

    hit_increment = logodds(0.7)
    hit_decrease = logodds(0.3/(NE-1))
    print("Hit increment", hit_increment, hit_decrease)

    test_odds[0] += hit_increment
    for i in range(1, NE):
        test_odds[i] += hit_decrease

    print(test_odds[0])
    print(probability(test_odds))
    print("Sum lo", np.sum(test_odds))
    print("Sum", np.sum(probability(test_odds,)))

    other = (1 - probability(test_odds[0])) / (NE-1)
    print(probability(logodds(other) - logodds(1/NE))/NE)

if __name__ == '__main__':
    main()