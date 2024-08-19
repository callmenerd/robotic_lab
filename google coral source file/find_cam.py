import subprocess

def find_cam(name):
    output = subprocess.run(["cam_with_name", name], capture_output=True)
    dev_name = str(output.stdout)

    def find_first_number(text):
        for char in text:
            if char.isdigit():
                return char
        return None

    return find_first_number(dev_name)

print(find_cam("Pro"))