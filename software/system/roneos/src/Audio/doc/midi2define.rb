#!/usr/bin/env ruby

require "pp"

# Add is_integer? method to string
class String
    def is_integer?
        # if we can't make an integer, we aren't an integer
        Integer(self) rescue return false
        true
    end
end


# Open the file (passed in as first argument) for reading)
file = File.new(ARGV[0],"r")

# Read in all of the file to a string
fileContents = file.read

# Remove any \r or \n characters and replace with a space
# The tr! means transform, in place. tr would create a new string.
fileContents.tr!("\r\n", " ")

# Remove any garbage clauses--i.e., [ EXC 3] or somesuch.
fileContents.gsub!(/(\[[\w\s\-\.]*\])?/,"")

# Spilt the string up by digits, setting aside comment blocks
numberRegEx = /\d+/ # numbers consist of at least one digit
patchNameRegEx = /[A-Za-z\s\-_]*/ # a patch name consists of some non-digit characters, whitespace, and maybe a _ or -
patchCommentRegEx = /\([\d\D\s\-_]*?\)/ # a patch comment is some string (digits or alphabet characters) inside of a parens

fullNoCommentsRegEx = /#{numberRegEx}+?\s*#{patchNameRegEx}/
fullWithCommentsRegEx = /#{numberRegEx}+?\s*#{patchNameRegEx}\s*#{patchCommentRegEx}+/

fileContentsProcessed = fileContents.scan /#{fullWithCommentsRegEx}|#{fullNoCommentsRegEx}/

# We now have an array of strings with the full patch names. BUT, any names with trailing numbers are truncated, and some
# strings that we have are really just those trailing numbers.
# So, we now scan through and push any pure number string back into the previous string.
lastString = ""
out = []
fileContentsProcessed.each do |patch|
    # Test if the patch is really just a misplaced number
    if patch.is_integer?
        # If it is, add it to the previous last string.
       lastString << " " + patch
    else
        # If it isn't, we've found a new patch name, and should put it out.
        lastString = patch
        out.push lastString
    end
end

# Sort the results, because we can.
out.sort! { |x,y| x.to_i <=> y.to_i }

# At this point, out contains properly formatted entries. Now, we just need to make them into C defines.
# This function assumes a properly formatted patch string.
def make_c_define str
    name = str.partition( "#{str.to_i}" )
    return "#define " + name[2].strip!.upcase!.tr_s("()\-","").tr_s("+ ","_") + " (#{name[1]})"
end

# convert the array of patch names to #defines
out.map!{ |str| make_c_define str }

# Print out to screen, so we can inspect and cat to a file
puts out.join "\n"

