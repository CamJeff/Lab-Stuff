import re

# Read TOML data from file
with open('data/values_cache.toml', 'r') as file:
    toml_data = file.read()

# Function to format float strings to 6 decimal places
def format_floats_in_string(match):
    num_str = match.group(0)
    formatted = f"{float(num_str):0.6f}"
    return formatted

# Regex pattern to find floats (including scientific notation)
float_pattern = r'(?<![\w.])[-+]?(?:\d*\.\d+|\d+\.\d*|\d+)(?:[eE][-+]?\d+)?(?![\w.])'

# Replace all floats in the string
formatted_toml_data = re.sub(float_pattern, format_floats_in_string, toml_data)

# Save the formatted data back to file
with open('data/values_cache_formatted.toml', 'w') as file:
    file.write(formatted_toml_data)

# Confirm completion
"Formatting complete. Output written to data/values_cache_formatted.toml."

