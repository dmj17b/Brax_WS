import numpy as np

def CalcMasses(total_mass):
    n_legs = 4
    n_wheels = 8
    body = 0.4*total_mass
    head = 0.2*total_mass
    thigh = (0.15/4)*total_mass
    shin = 0.2*(0.25/4)*total_mass
    wheel = 0.4*(0.25/4)*total_mass
    
    masses = {
        "body": body,
        "head": head,
        "thigh": thigh,
        "shin": shin,
        "wheel": wheel
    }
    print("Body mass: ", body)
    print("Head mass: ", head)
    print("Thigh mass: ", thigh)
    print("Shin mass: ", shin)
    print("Wheel mass: ", wheel)
    
    check_sum = body+head+n_legs*(thigh+shin)+n_wheels*wheel
    print(check_sum)

    return masses

def CalcTotal(masses):
    check_sum = masses["body"]+masses["head"]+4*(masses["thigh"]+masses["shin"])+8*masses["wheel"]
    print("Check sum: ", check_sum)
    return check_sum

def main():
    total_mass = 90
    print("Total mass: ", total_mass)
    CalcMasses(total_mass)
    masses = {
        "body": 30,
        "head": 15,
        "thigh": 3,
        "shin": 1,
        "wheel": 2
    }
    CalcTotal(masses)


if __name__ == '__main__':
    main()