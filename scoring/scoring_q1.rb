#!/usr/bin/env ruby

require 'nokogiri'
require 'matrix'

# Calculate position distance between two matrices
def matDistance(matA, matB)
  a = Vector[matA[0, 3],
             matA[1, 3],
             matA[2, 3]]

  b = Vector[matB[0, 3],
             matB[1, 3],
             matB[2, 3]]

  return (a - b).magnitude()
end

# Get a matrix from pose elements
def matFromPose(x, y, z, roll, pitch, yaw)

  m00 = Math.cos(yaw) * Math.cos(pitch)
  m01 = Math.cos(yaw) * Math.sin(pitch) * Math.sin(roll) - Math.sin(yaw) * Math.cos(roll)
  m02 = Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll) + Math.sin(yaw) * Math.sin(roll)
  m03 = x

  m10 = Math.sin(yaw) * Math.cos(pitch)
  m11 = Math.sin(yaw) * Math.sin(pitch) * Math.sin(roll) + Math.cos(yaw) * Math.cos(roll)
  m12 = Math.sin(yaw) * Math.sin(pitch) * Math.cos(roll) - Math.cos(yaw) * Math.sin(roll)
  m13 = y

  m20 = -Math.sin(pitch)
  m21 = Math.cos(pitch) * Math.sin(roll)
  m22 = Math.cos(pitch) * Math.cos(roll)
  m23 = z

  return Matrix[
           [m00, m01, m02, m03],
           [m10, m11, m12, m13],
           [m20, m21, m22, m23],
           [0.0, 0.0, 0.0, 1.0]
         ]
end

# Get a matrix from a chunk and a path
def getMatrix(chunk, path)
  pose = chunk.xpath(path)

  if pose.size != 1
    printf("Couldn't find pose for path %s\n", path)
    return
  end

  parts = pose.text().split
  if parts.size != 6
    printf("Malformed pose for path %s\n", path)
    return
  end

  return matFromPose(parts[0].to_f,
                     parts[1].to_f,
                     parts[2].to_f,
                     parts[3].to_f,
                     parts[4].to_f,
                     parts[5].to_f)
end

class Time
  def initialize
    @sec = 0
    @nsec = 0
  end

  def eql?(other)
    return @sec == other.sec && @nsec == other.nsec
  end

  def hash
    return @sec.to_f + @nsec.to_f * 1e-9
  end

  def -(other)
    result = Time.new
    result.sec = @sec - other.sec
    result.nsec = @nsec - other.nsec
    result.correct
    return result
  end

  def >=(other)
    return @sec.to_i >= other.sec && @nsec.to_i >= other.nsec
  end

  def correct
    if (@sec > 0 && @nsec < 0)
      n = (@nsec / 1000000000).abs.to_i + 1
      @sec -= n
      @nsec += n * 1000000000
    end
    if (@sec < 0 && @nsec > 0)
      n = Math.abs(@nsec / 1000000000).to_i + 1
      @sec += n
      @nsec -= n * 1000000000
    end

    @sec += (@nsec / 1000000000).to_i
    @nsec = (@nsec % 1000000000).to_i
  end

  attr_accessor :sec
  attr_accessor :nsec
end

class Color
  def initialize
    @r = 0.0
    @g = 0.0
    @b = 0.0
    @a = 1.0
  end

  def ==(other)
    return @r == other.r && @g == other.g && @b == other.b && @a == other.a
  end

  def difference(other)
    return Math.sqrt((@r-other.r) * (@r-other.r) +
                     (@g-other.g) * (@g-other.g) +
                     (@b-other.b) * (@b-other.b) +
                     (@a-other.a) * (@a-other.a))
  end

  attr_accessor :r
  attr_accessor :g
  attr_accessor :b
  attr_accessor :a
end

class State
  def initialize(file)
    # Open and read the log file
    doc = Nokogiri::XML(File.open(file))

    # Get the first chunk
    chunks = doc.xpath("//gazebo_log/chunk")
    if chunks.size < 2
      puts "State log file has less than 2 entries."
      return
    end

    chunk = Nokogiri::XML(chunks[0].text)

    # Get console pose in world frame
    consoleWorldMat = getMatrix(chunk, "//sdf/world/model[@name='console1']/pose")

    # printf("Console world pos [%f %f %f]\n",
    #        consoleWorldMat[0, 3], consoleWorldMat[1, 3], consoleWorldMat[2, 3])

    # Read all the light positions
    @lightMats = Hash.new
    for i in 1..44

      # Light in console frame (local frame because this is not from states)
      lightLocalMat = getMatrix(chunk, "//sdf/world/model/link/visual[@name='light#{i}']/pose")

      # Light in world frame
      @lightMats[i] = consoleWorldMat * lightLocalMat

      # printf("Light [%i] world position [%f %f %f]\n", i,
      #        *@lightMats[i][0, 3], *@lightMats[i][1, 3], *@lightMats[i][2, 3])
    end

    # Create hash of the head pose over time
    @headMats = Hash.new
    for i in 1..chunks.size-1
      chunk = Nokogiri::XML(chunks[i].text)

      # Read the sim time
      parts = chunk.xpath("//sdf/state/sim_time").text.split
      time = Time.new
      time.sec = parts[0]
      time.nsec = parts[1]

      # Read the head pose in world frame (world frame because they're from states)
      @headMats[time] = getMatrix(chunk, "//sdf/state/model/link[@name='upperNeckPitchLink']/pose")

      # printf("Time[%d.%d] Pose[%f %f %f]\n", time.sec, time.nsec,
      #         @headMats[time][0, 3], @headMats[time][1, 3], @headMats[time][2, 3])
    end
  end

  # Return the matrix of a light in the world frame, according to the index
  def lightMat(index)
    mat = Matrix.identity(4)
    if @lightMats.has_key?(index)
      mat = @lightMats[index]
    end
    return mat
  end

  # Return the last matrix of the head in the world frame before the given time
  def headMat(time)

    @headMats.each do |key, mat|
      if key >= time
        # printf("Time wanted[%d.%d] Time found[%d.%d]\n", time.sec, time.nsec, key.sec, key.nsec)
        return mat
      end
    end

    return Matrix.identity(4)
  end
end

if ARGV.size != 2
  puts "Usage: scoring.rb <answer.log> <state.log>"
  exit -1
end

qualLog = ARGV[0]
stateLog = ARGV[1]

if !File.file?(qualLog)
  puts "Invalid qual1 log file"
  exit 0
end

if !File.file?(stateLog)
  puts "Invalid state log file"
  exit 0
end

black = Color.new

# Read all the state information
state = State.new(stateLog)

start = Time.new
currentTime = Time.new
currentColor = Color.new
latestLightMat = Matrix.identity(4)
lightIndex = -1

colorTolerance = 0.25
posTolerance = 0.05
numCorrect = 0
colorTotalError = 0
posTotalError = 0

File.open(qualLog).each do |line|

  # Skip lines that begin with "#"
  if line =~ /^#/
    next
  end

  # Split the line
  parts = line.split

  # Process the "start" line
  if line =~ /^start/
    if parts.size != 3
      puts "Invalid 'start' line, exiting: "
      puts line
      exit 0
    end

    start.sec = parts[1].to_i
    start.nsec = parts[2].to_i
  end

  # Process the "switch" line
  if line =~ /^switch/
    if parts.size != 8
      puts "Invalid 'switch' line, exiting: "
      puts line
      exit 0
    end

    currentColor.r = parts[2].to_f
    currentColor.g = parts[3].to_f
    currentColor.b = parts[4].to_f
    currentColor.a = parts[5].to_f

    lightTime = Time.new
    lightTime.sec = parts[6].to_i
    lightTime.nsec = parts[7].to_i
    currentTime = lightTime

    # If not black, then set the light index
    if currentColor != black
      lightIndex = parts[1].to_i
      latestLightMat = state.lightMat(lightIndex)

      # printf("Switch: Time[%4.2f] Color[%2.1f %2.1f %2.1f] Light world pos[%6.4f %6.4f %6.4f] Index[%d]\n",
      #        lightTime.sec + lightTime.nsec * 1e-9,
      #        currentColor.r, currentColor.g, currentColor.b,
      #        latestLightMat[0, 3], latestLightMat[1, 3], latestLightMat[2, 3],
      #        lightIndex)
    end
  end

  # Process the "answer" line
  if line =~ /^answer/
    if parts.size != 9
      puts "Invalid 'answer' line, exiting: "
      puts line
      exit 0
    end

    # Answer time
    answerTime = Time.new
    answerTime.sec = parts[7].to_i
    answerTime.nsec = parts[8].to_i
    currentTime = answerTime

    # Answer color
    answerColor = Color.new
    answerColor.r = parts[4].to_f
    answerColor.g = parts[5].to_f
    answerColor.b = parts[6].to_f

    # Compute difference to previous light color
    colorError = answerColor.difference(currentColor)
    colorTotalError += colorError

    colorFail = colorError > colorTolerance

    # Answer pose in head frame
    answerLocalMat = matFromPose(parts[1].to_f, parts[2].to_f, parts[3].to_f, 0, 0, 0)

    # Head matrix in world frame at this time
    headMat = state.headMat(answerTime)

    # Amswer pose in world frame
    answerMat = headMat * answerLocalMat

    # Compute distance between the light pose and the answer
    posError = matDistance(latestLightMat, answerMat)
    posTotalError += posError

    posFail = posError > posTolerance

    # printf("Answer: Time[%4.2f] Color[%2.1f %2.1f %2.1f] Answer local pos[%6.4f %6.4f %6.4f] Head Pos [%6.4f %6.4f %6.4f] Answer world pos[%6.4f %6.4f %6.4f] Position error [%6.4f] Color error [%6.4f]\n",
    #        answerTime.sec + answerTime.nsec * 1e-9,
    #        answerColor.r, answerColor.g, answerColor.b,
    #        answerLocalMat[0, 3], answerLocalMat[1, 3], answerLocalMat[2, 3],
    #        headMat[0, 3], headMat[1, 3], headMat[2, 3],
    #        answerMat[0, 3], answerMat[1, 3], answerMat[2, 3],
    #        posError, colorError)

    # If color or position are wrong, reset
    if (colorFail || posFail)
      printf("[FAIL]    ")
    else
      printf("[SUCCESS] ")
    end

    # Print answer summary
    printf("Color:    real   [%2.4f %2.4f %2.4f]\n",
           currentColor.r, currentColor.g, currentColor.b)

    printf("                    answer [%2.4f %2.4f %2.4f]\n",
           answerColor.r, answerColor.g, answerColor.b)

    printf("                    error  [%2.6f]\n", colorError)

    printf("          Position: real   [%2.4f %2.4f %2.4f]\n",
           latestLightMat[0, 3], latestLightMat[1, 3], latestLightMat[2, 3])

    printf("                    answer [%2.4f %2.4f %2.4f]\n",
           answerMat[0, 3], answerMat[1, 3], answerMat[2, 3])

    printf("                    error  [%2.6f]\n", posError)

    if (colorFail || posFail)
      colorTotalError = 0
      posTotalError = 0
      numCorrect = 0
      next
    end

    numCorrect += 1

    # Get the first 10 correct in a row
    if (numCorrect == 10)
      break
    end
  end
end

# Calculate duration
duration = Time.new
duration = currentTime - start

# 10 lights correct in a row?
success = numCorrect == 10 ? "Yes" : "No"

# Calculate score
colorMax = Math::sqrt(colorTolerance)
posMax = Math::sqrt(posTolerance)

colorScore = colorMax
posScore = posMax
if (numCorrect > 0)
  colorAvgError = colorTotalError / numCorrect
  colorScore = colorAvgError / colorMax

  posAvgError = posTotalError / numCorrect
  posScore = posAvgError / posMax
end

colorWeight = 0.5
posWeight = 0.5
score = colorScore * colorWeight + posScore * posWeight

printf("--------------------------------\n")
printf("Duration: %d.%d\n", duration.sec, duration.nsec)
printf("Success: " + success + "\n")
printf("Color score: %1.6f\n", colorScore)
printf("Position score: %1.6f\n", posScore)
printf("Total score: %1.6f\n", score)
printf("--------------------------------\n")

