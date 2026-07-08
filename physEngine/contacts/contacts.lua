local ContactGenerators = {}

function ContactGenerators.sortByPenetration(contact1, contact2)
	return contact1.penetration > contact2.penetration
end

return ContactGenerators
