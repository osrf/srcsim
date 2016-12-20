#!/usr/bin/env ruby

require 'nokogiri'
require 'matrix'

def matDistance(matA, matB)
  a = Vector[matA[0, 3],
             matA[1, 3],
             matA[2, 3]]

  b = Vector[matB[0, 3],
             matB[1, 3],
             matB[2, 3]]

  return (a - b).magnitude()
end

class Quaternion
  def initialize
    @x = 0
    @y = 0
    @z = 0
    @w = 1
  end

  def set(roll, pitch, yaw)
    phi, the, psi = 0.0

    phi = roll / 2.0
    the = pitch / 2.0
    psi = yaw / 2.0

    @w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) +
      Math.sin(phi) * Math.sin(the) * Math.sin(psi)
    @x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) -
      Math.cos(phi) * Math.sin(the) * Math.sin(psi)
    @y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) +
      Math.sin(phi) * Math.cos(the) * Math.sin(psi)
    @z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) -
      Math.sin(phi) * Math.sin(the) * Math.cos(psi)
    normalize()
  end

  def *(other)
    result = Quaternion.new
    result.w = @w*other.w-@x*other.x-@y*other.y-@z*other.z
    result.x = @w*other.x+@x*other.w+@y*other.z-@z*other.y
    result.y = @w*other.y-@x*other.z+@y*other.w+@z*other.x
    result.z = @w*other.z+@x*other.y-@y*other.x+@z*other.w
    return result
  end

  def inverse
    s = 0;
    q = Quaternion.new
    q.w = @w
    q.x = @x
    q.y = @y
    q.z = @z

    s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

    if s == 0
      q.w = 1.0
      q.x = 0.0
      q.y = 0.0
      q.z = 0.0
    else
      q.w =  q.w / s
      q.x = -q.x / s
      q.y = -q.y / s
      q.z = -q.z / s
    end

    return q
  end

  def normalize
    s = 0.0;

    s = Math.sqrt(@w * @w + @x * @x + @y * @y + @z * @z);

    if s == 0
      @w = 1.0;
      @x = 0.0;
      @y = 0.0;
      @z = 0.0;
    else
      @w /= s;
      @x /= s;
      @y /= s;
      @z /= s;
    end
  end

  attr_accessor :x
  attr_accessor :y
  attr_accessor :z
  attr_accessor :w
end

class Pose
  def initialize
    @p = Vector[]
    @q = Quaternion.new
  end

  def set(x, y, z, roll, pitch, yaw)
    @p = Vector[x, y, z]
    @q.set(roll, pitch, yaw)
  end

  def mat

    @q.normalize()

    m00 = 1 - 2 * @q.y * @q.y - 2 * @q.z * @q.z
    m01 = 2 * @q.x * @q.y - 2 * @q.z * @q.w
    m02 = 2 * @q.x * @q.z + 2 * @q.y * @q.w
    m03 = @p[0]

    m10 = 2 * @q.x * @q.y + 2 * @q.z * @q.w
    m11 = 1 - 2 * @q.x * @q.x - 2 * @q.z * @q.z
    m12 = 2 * @q.y * @q.z - 2 * @q.x * @q.w
    m13 = @p[1]

    m20 = 2 * @q.x * @q.z - 2 * @q.y * @q.w
    m21 = 2 * @q.y * @q.z + 2 * @q.x * @q.w
    m22 = 1 - 2 * @q.x * @q.x - 2 * @q.y * @q.y
    m23 = @p[2]

    return Matrix[
             [m00, m01, m02, m03],
             [m10, m11, m12, m13],
             [m20, m21, m22, m23],
             [0.0, 0.0, 0.0, 1.0]
           ]
  end

  def +(other)
    result = Pose.new

    tmp = Quaternion.new
    tmp.w = 0.0
    tmp.x = other.p[0]
    tmp.y = other.p[1]
    tmp.z = other.p[2]
    tmp = @q * (tmp * @q.inverse)

    result.p = Vector[@p[0] + tmp.x, @p[1] + tmp.y, @p[2] + tmp.z]
    result.q = other.q * @q

    return result
  end

  def distance(other)
    return @p.distance(other.p)
  end

  attr_accessor :p
  attr_accessor :q
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

    @lights = Hash.new

    chunk = Nokogiri::XML(chunks[0].text)

    # Get console pose in world frame
    @consolePose = Pose.new
    pose = chunk.xpath("//sdf/world/model[@name='console1']/pose")
    if pose.size == 1
      parts = pose.text().split
      @consolePose.set(parts[0].to_f,
                       parts[1].to_f,
                       parts[2].to_f,
                       parts[3].to_f,
                       parts[4].to_f,
                       parts[5].to_f)
    end
    # printf("Console world pose [%f %f %f %f %f %f %f]\n",
    #        @consolePose.p.x, @consolePose.p.y, @consolePose.p.z,
    #        @consolePose.q.x, @consolePose.q.y, @consolePose.q.z, @consolePose.q.w)

    # Read all the light positions
    for i in 1..44

      # Light in console frame (local frame because this is not from states)
      light = chunk.xpath("//sdf/world/model/link/visual[@name='light#{i}']")
      lightPoseParts = light.xpath(".//pose").text().split

      lightLocalPose = Pose.new
      lightLocalPose.set(lightPoseParts[0].to_f,
                         lightPoseParts[1].to_f,
                         lightPoseParts[2].to_f,
                         lightPoseParts[3].to_f,
                         lightPoseParts[4].to_f,
                         lightPoseParts[5].to_f)

      # Light in world frame
      @lights[i] = @consolePose.mat * lightLocalPose.mat

      # printf("Light [%i] world position [%f %f %f]\n", i,
      #        *@lights[i][0, 3], *@lights[i][1, 3], *@lights[i][2, 3])
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
      headPose = Pose.new
      head = chunk.xpath(
        "//sdf/state/model/link[@name='upperNeckPitchLink']/pose")
      if head.size == 1
        parts = head.text().split
        headPose.set(parts[0].to_f, parts[1].to_f, parts[2].to_f,
                     parts[3].to_f, parts[4].to_f, parts[5].to_f)
      end

      @headMats[time] = headPose.mat
      # printf("Time[%d.%d] Pose[%f %f %f]\n", time.sec, time.nsec,
      #         @headMats[time][0, 3], @headMats[time][1, 3], @headMats[time][2, 3])
    end
  end

  # Return the matrix of a light in the world frame, according to the index
  def lightMat(index)
    mat = Matrix.identity(4)
    if @lights.has_key?(index)
      mat = @lights[index]
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
error = 0
colorWeight = 1
posWeight = 1

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

      printf("Switch: Time[%4.2f] Color[%2.1f %2.1f %2.1f] Light world pos[%6.4f %6.4f %6.4f] Index[%d]\n",
             lightTime.sec + lightTime.nsec * 1e-9,
             currentColor.r, currentColor.g, currentColor.b,
             latestLightMat[0, 3], latestLightMat[1, 3], latestLightMat[2, 3],
             lightIndex)
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
    error += colorWeight * colorError

    # Answer pose in head frame
    answerLocalPose = Pose.new
    answerLocalPose.set(parts[1].to_f, parts[2].to_f, parts[3].to_f, 0, 0, 0)

    # Head matrix in world frame at this time
    headMat = state.headMat(answerTime)

    # Amswer pose in world frame
    answerMat = headMat * answerLocalPose.mat

    # Compute distance between the light pose and the answer
    posError = matDistance(latestLightMat, answerMat)
    error += posWeight * posError

    printf("Answer: Time[%4.2f] Color[%2.1f %2.1f %2.1f] Answer local pos[%6.4f %6.4f %6.4f] Head Pos [%6.4f %6.4f %6.4f] Answer world pos[%6.4f %6.4f %6.4f] Position error [%6.4f] Color error [%6.4f]\n",
           answerTime.sec + answerTime.nsec * 1e-9,
           answerColor.r, answerColor.g, answerColor.b,
           answerLocalPose.p[0], answerLocalPose.p[1], answerLocalPose.p[2],
           headMat[0, 3], headMat[1, 3], headMat[2, 3],
           answerMat[0, 3], answerMat[1, 3], answerMat[2, 3],
           posError, colorError)
  end
end

duration = Time.new
duration = currentTime - start
printf("Duration: %d.%d Error: %f\n", duration.sec, duration.nsec, error)
