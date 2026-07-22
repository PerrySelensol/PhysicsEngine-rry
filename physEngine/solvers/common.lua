local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")

--[=============================================================================]--

local solverCommons = {}

-- Generate an arbitary basis with a given fixed axis
local generateOrthoBasis; do
	local Y1, Y2 = vec(1,0,0), vec(0,1,0)
	local abs, mat3 = math.abs, matrices.mat3
	function generateOrthoBasis(fixedX)
		local genZ = fixedX^((abs(fixedX..Y1) < 0.75) and Y1 or Y2)

		genZ:normalize()
		local genY = genZ^fixedX

		return mat3(fixedX, genY, genZ)
	end
end

local function crossMat(v) -- Cross product matrix so that crossMat(v) * u = v ^ u
	return matrices.mat3(
		vec(0,		v.z,	-v.y),
		vec(-v.z,	0,		v.x ),
		vec(v.y,	-v.x,	0   )
	)
end

function solverCommons.getSeparatingVel(A, B, contactPointA, contactPointB, contactMatrix)
	local totalSepVel = A.vel + (A.rot ^ contactPointA)
	if B then
		totalSepVel = totalSepVel - (B.vel + (B.rot ^ contactPointB))
	end
	return contactMatrix:transposed() * totalSepVel
end

local I3 = matrices.mat3()
function solverCommons.prepareContact(contact)
	if contact.type ~= "contact" then return end
	contact.contactMatrix = generateOrthoBasis(contact.contactNormal)
	contact.accumulatedNormalImpulse = 0

	local totalInertia

	local linearInertiaA = I3 * contact.A.inverseMass
	local angularInertiaA

	local linearInertiaB
	local angularInertiaB

	do
		local cross_relativeContactPointA = crossMat(contact.A.oriMat * contact.contactPointA)

		local angularImpulsePerLinearImpulse = cross_relativeContactPointA * contact.contactMatrix
		local rotPerUnit = contact.A.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointA * rotPerUnit * -1

		angularInertiaA = contact.contactMatrix:transposed() * velPerUnit

		totalInertia = angularInertiaA + linearInertiaA
	end

	if contact.B then
		linearInertiaB = I3 * contact.B.inverseMass

		local cross_relativeContactPointB = crossMat(contact.B.oriMat * contact.contactPointB)

		local angularImpulsePerLinearImpulse = cross_relativeContactPointB * contact.contactMatrix
		local rotPerUnit = contact.B.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointB * rotPerUnit * -1

		angularInertiaB = contact.contactMatrix:transposed() * velPerUnit

		totalInertia = totalInertia + angularInertiaB + linearInertiaB
	end

	contact.totalInertia = totalInertia

	contact.linearInertiaA = linearInertiaA
	contact.angularInertiaA = angularInertiaA

	contact.linearInertiaB = linearInertiaB
	contact.angularInertiaB = angularInertiaB

end

return solverCommons