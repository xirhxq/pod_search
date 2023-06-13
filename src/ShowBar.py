class ShowBar:
    def __init__(self, _max, _min):
        self.right = _max
        self.left = _min

        self.grid_num_total = 50

    def show(self, val, str=''):
        percentage = (val - self.left) / (self.right - self.left)
        grid_num = int(percentage * self.grid_num_total)

        print(f'{str}: {val:.1f} \n{self.left:.1f}' + '[' + '#' * grid_num + ' ' * (self.grid_num_total - grid_num) + ']' + f'{self.right:.1f}')