# Project conventions

Avoid import aliases. When a member name is ambiguous, import only its namespace
and qualify references through that namespace instead of importing individual members.

Do not combine a namespace and its members in `{self, ...}` imports. Import only
the namespace and use `::` references for its members.

Keep items private unless production code outside their module needs them. Tests
are not a reason to expose implementation details: put such tests in the owning
module or remove them. Keep shared fixture builders in test support modules.
Don't use pub(crate) when simply pub works.
